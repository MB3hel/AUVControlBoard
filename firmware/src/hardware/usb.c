/*
 * Copyright 2022 Marcus Behel
 * 
 * This file is part of AUVControlBoard-Firmware.
 * 
 * AUVControlBoard-Firmware is free software: you can redistribute it and/or modify it under the terms of the GNU 
 * General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 * 
 * AUVControlBoard-Firmware is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even 
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for 
 * more details.
 * 
 * You should have received a copy of the GNU General Public License along with AUVControlBoard-Firmware. If not, see 
 * <https://www.gnu.org/licenses/>. 
 * 
 */

#include <hardware/usb.h>
#include <framework.h>
#include <limits.h>
#include <string.h>
#include <FreeRTOS.h>
#include <app.h>


// Special bytes for protocol
#define START_BYTE          253
#define END_BYTE            254
#define ESCAPE_BYTE         255

#if defined(CONTROL_BOARD_V2)
extern RTC_HandleTypeDef hrtc;
#endif


bool usb_initialized = false;


#if defined(CONTROL_BOARD_V1) || defined(CONTROL_BOARD_V2)

#include <tusb.h>

void usb_init(void){
#if defined(CONTROL_BOARD_V1)
    // Set interrupt to highest allowed priority
    NVIC_SetPriority(USB_OTHER_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(USB_SOF_HSOF_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(USB_TRCPT0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(USB_TRCPT1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    // Pins configured by generated code
    // Clock config handled by generated code

    // Start TinyUSB
    tud_init(BOARD_TUD_RHPORT);
#elif defined(CONTROL_BOARD_V2)
    // Set interrupt to highest allowed priority
    NVIC_SetPriority(OTG_FS_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    // Pins configured by generated code

    // Enable clock to USB FS peripheral
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

    // Disable VBUS sense
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;

    // Start TinyUSB
    tud_init(BOARD_TUD_RHPORT);
#endif
    usb_initialized = true;
}


void usb_process(void){
    tud_task();
}

unsigned int usb_avail(void){
    return tud_cdc_available();
}

uint8_t usb_read(void){
    return tud_cdc_read_char();
}

void usb_write(uint8_t b){
    if(!tud_cdc_write_char(b)){
        tud_cdc_write_flush();
        tud_cdc_write_char(b);
    }
}

void usb_flush(void){
    tud_cdc_write_flush();
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts){
    (void)rts;

    // Run when line state changes
    // DTR = Data Terminal Ready
    // RTS = Ready to Send
    // DTR usually set when terminal connected

    // sam-ba upload protocol uses a 1200bps "touch" to trigger a reset
    // Handle this as expected
    // Not strictly necessary, but prevents having to press reset button to program
    // 1200bps "touch" means opening the port at 1200bps then closing it again (quickly)
    // Nothing is implemented here with timing. It just boots to bootloader when
    // a 1200bps connection is closed
    if (!dtr && itf == 0) {
        app_handle_uart_closed();

        cdc_line_coding_t coding;
        tud_cdc_get_line_coding(&coding);
        if (coding.bit_rate == 1200){
#if defined(CONTROL_BOARD_V1)
            // Special things to reboot to bootloader instead of main program
            // Must match bootloader. Taken from Adafruit/ArduinoCore-samd Reset.cpp
            // THIS IS SPECIFIC TO ITSY BITSY M4!!!
            #define DOUBLE_TAP_MAGIC             0xf01669efUL
            #define BOOT_DOUBLE_TAP_ADDRESS     (HSRAM_ADDR + HSRAM_SIZE - 4)
            volatile unsigned long *a = (unsigned long *)BOOT_DOUBLE_TAP_ADDRESS;
            *a = DOUBLE_TAP_MAGIC;

            // Reset the system now
            NVIC_SystemReset();
            while(1);
#elif defined (CONTROL_BOARD_V2)
            // Reboot by setting a value in a backup register (0)
            // After system reset, if this is set will branch to bootloader
            // This ensures that system is in reset state when entering bootloader
            HAL_PWR_EnableBkUpAccess();
            HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x3851FDEB);
            HAL_PWR_DisableBkUpAccess();
            NVIC_SystemReset();
            while(1);
#endif
        }
    }
}

#endif // CONTROL_BOARD_V1 || CONTROL_BOARD_V2


#if defined(CONTROL_BOARD_SIM)

#include <stdio.h>
#include <stdint.h>
#include <util/circular_buffer.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <sys/poll.h>

#define USB_WB_SIZE 128
#define USB_RB_SIZE 128

static uint8_t write_buf[USB_WB_SIZE];
static unsigned int write_buf_pos = 0;

static uint8_t read_buff_arr[USB_RB_SIZE];
static circular_buffer read_buf;

// Both buffers must be mutex protected as different threads may both change
// the count / position variables
static SemaphoreHandle_t write_mutex;
static SemaphoreHandle_t read_mutex;

static int server_fd;
static int client_fd = -1;
static struct sockaddr_in client_addr;
static socklen_t client_addr_len;

bool usb_setup(int port){
    // Setup socket
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if(server_fd == -1){
        fprintf(stderr, "Failed to create socket. Error code: %d\n", errno);
        return false;
    }
    struct sockaddr_in a;
    memset(&a, '0', sizeof(a));
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = htons(port);
    if(bind(server_fd, (struct sockaddr *)&a, sizeof(a)) == -1){
        fprintf(stderr, "Failed to bind socket. Error code: %d\n", errno);
        return false;
    }
    if(listen(server_fd, 1) == -1){
        fprintf(stderr, "Failed to listen on socket. Error code: %d\n", errno);
        return false;
    }

    client_fd = -1;

    // Success
    return true;
}

void usb_init(void){
    write_buf_pos = 0;
    cb_init(&read_buf, read_buff_arr, USB_RB_SIZE);

    // Mutexes are created ready to be taken
    write_mutex = xSemaphoreCreateMutex();
    read_mutex = xSemaphoreCreateMutex();

    usb_initialized = true;
}

void usb_process(void){
    // Potentially need to repeat the wait for connection / read sequence
    while(1){

        // Wait until connection if none
        while(client_fd < 0){
            // accept blocks so need to mask signals 
            // See: https://www.freertos.org/FreeRTOS-simulator-for-Linux.html
            sigset_t fullset, origset;
            sigfillset(&fullset);
            pthread_sigmask(SIG_SETMASK, &fullset, &origset);
            client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_addr_len);
            pthread_sigmask(SIG_SETMASK, &origset, NULL);
        }
        
        // Block until socket has data
        // Don't want to read data here. Just want to return from this function
        // Indicating to the caller that there is data ready to read using usb_read
        // Thus using the socket's buffering to hold the data
        // As such, poll is used
        struct pollfd pfd;
        pfd.fd = client_fd;
        pfd.events = POLLIN;

        // poll blocks so need to mask signals 
        // See: https://www.freertos.org/FreeRTOS-simulator-for-Linux.html
        sigset_t fullset, origset;
        sigfillset(&fullset);
        pthread_sigmask(SIG_SETMASK, &fullset, &origset);
        int res = poll(&pfd, 1, -1);
        pthread_sigmask(SIG_SETMASK, &origset, NULL);

        if(res < 0){
            // Poll error
            // Probably no recovery from this...
            fprintf(stderr, "CRITICAL ERROR CALLING POLL ON SOCKET!!!\n");
        }else if(res == 0){
            // Timeout. Just let the outer loop repeat
        }else{
            // Got an event for the client socket. Handle the event(s)
            if(pfd.revents == POLLIN){
                // Ready to read data
                // Return from this function now b/c there is data available for read.
                return;
            }else{
                // Some error on the FD. Probably client closed connection
                close(client_fd);
                client_fd = -1;
                // Loop repeats back to top waiting for new connection now.
            }
        }
    }
}

unsigned int usb_avail(void){
    if(client_fd < 0)
        return 0;
    size_t n = 0;
    ioctl(client_fd, FIONREAD, &n);
    return (unsigned int)n;
}

uint8_t usb_read(void){
    uint8_t b = 0;
    if(client_fd < 0)
        return b;
    if(usb_avail() == 0)
        return b;
    xSemaphoreTake(read_mutex, portMAX_DELAY);
    // read blocks while actually reading, so need to mask signals
    // See: https://www.freertos.org/FreeRTOS-simulator-for-Linux.html
    sigset_t fullset, origset;
    sigfillset(&fullset);
    pthread_sigmask(SIG_SETMASK, &fullset, &origset);
    if(read(client_fd, &b, 1) == -1){
        // Read error. Assume connection closed
        close(client_fd);
        client_fd = -1;
        // Note that if this occurs a garbage byte is returned from USB read
        // but that's ok because the parse will eventually discard it.
        // Same thing (essentially) as UART comm drop + noise on last byte before drop
    }
    pthread_sigmask(SIG_SETMASK, &origset, NULL);
    xSemaphoreGive(read_mutex);
    return b;
}

void usb_write(uint8_t b){
    if(client_fd < 0)
        return;

    xSemaphoreTake(write_mutex, portMAX_DELAY);
    write_buf[write_buf_pos] = b;
    write_buf_pos++;
    if(write_buf_pos == USB_WB_SIZE){
        xSemaphoreGive(write_mutex);
        usb_flush();
    }else{
        xSemaphoreGive(write_mutex);
    }
}

void usb_flush(void){
    if(client_fd < 0)
        return;
    
    // Writing to STDOUT will block, but not for long enough that it matters
    // Not really any longer than writing out over USB would take on real hardware
    xSemaphoreTake(write_mutex, portMAX_DELAY);
    if(write_buf_pos > 0){
        // read blocks while actually writing, so need to mask signals
        // See: https://www.freertos.org/FreeRTOS-simulator-for-Linux.html
        sigset_t fullset, origset;
        sigfillset(&fullset);
        pthread_sigmask(SIG_SETMASK, &fullset, &origset);
        if(write(client_fd, write_buf, write_buf_pos) == -1){
            // Write failed. Assume client has disconnected.
            close(client_fd);
            client_fd = -1;
        }
        pthread_sigmask(SIG_SETMASK, &origset, NULL);
        write_buf_pos = 0;
    }
    xSemaphoreGive(write_mutex);
}

#endif // CONTROL_BOARD_SIM
