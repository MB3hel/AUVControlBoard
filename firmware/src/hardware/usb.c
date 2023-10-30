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

// TODO: Buffers

// Mutexes to ensure usb_write and usb_read are thread safe
// Only used by RTOS threads
SemaphoreHandle_t write_mutex, read_mutex;

// Socket & thread stuff
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

    // Parallel threads are used for read / write from / to socket
    // This prevents unexpected blocking behavior with RTOS threads
    // Things go poorly when RTOS threads invoke syscalls (blocks entire program)
    // So generally, keep syscalls to parallel threads
    // These parallel threads need signals masked though to prevent issues with RTOS function
    // See: https://www.freertos.org/Documentation/api-ref/POSIX/index.html
    // Since pthread_create makes a child inheriting the parent's sigmask, mask all here before creation
    // This is a portable way of ensuring that created thread's sigmask is correct at creation
    // Then, after creation, restore mask here so this still works as expected by RTOS port.
    // TODO

    // Success
    return true;
}

void usb_init(void){
    write_mutex = xSemaphoreCreateMutex();
    read_mutex = xSemaphoreCreateMutex();

    // TODO: Initialize RTOS stuff, socket stuff, and threads (in that order)
    // Note that usb_setup must be combined into here
    // necessary so that RTOS semaphores can be ready before threads are running
    // Use of RTOS semaphores (NOT MUTEXES!!!) in BG threads must use FromISR
    // functions and not actually yield

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
    return CB_AVAIL_READ(&read_buf);
}

uint8_t usb_read(void){
    if(client_fd == -1){
        // No connection. Ignore IO.
        return 0;
    }

    xSemaphoreTake(read_mutex, portMAX_DELAY);
    // TODO: Take semaphore zero timeout
    // If timeout, no data return garbage
    // Else get element from buffer and return it
    // Similar idea to what is described for indices for usb_write
    xSemaphoreGive(read_mutex);
}

void usb_write(uint8_t b){
    if(client_fd == -1){
        // No connection. Ignore IO.
        return;
    }

    xSemaphoreTake(write_mutex, portMAX_DELAY);
    // TODO: Put data in buffer
    // TODO: Give semaphore
    // TODO: This must be circular buffer. RTOS threads read from write buffer so use ridx
    // TODO: Background threads write into buffer, so use widx.
    // TODO: Only shared thing is counting semaphore which both can use b/c supports FromISR
    xSemaphoreGive(write_mutex);
}

void usb_flush(void){
    // Data is written realtime
    // No control over flushing in this implementation
    return;
}

#endif // CONTROL_BOARD_SIM
