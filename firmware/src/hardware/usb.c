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


#if defined(CONTROL_BOARD_SIM_LINUX) || defined(CONTROL_BOARD_SIM_MACOS)

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <util/circular_buffer.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <sys/poll.h>
#include <pthread.h>
#include <sys/ioctl.h>


// Buffers
// Write buffer is linear (it is always emptied in full)
// Read buffer is circular (ring) buffer
#define USB_WB_SIZE 128
#define USB_RB_SIZE 128
static uint8_t write_buf[USB_WB_SIZE];
static unsigned int write_buf_pos;
static uint8_t read_buf_arr[USB_RB_SIZE];
static circular_buffer read_buf;
static SemaphoreHandle_t avail_to_read_sem;

// Socket & thread stuff
static int server_fd;
static int client_fd = -1;
static struct sockaddr_in client_addr;
static socklen_t client_addr_len;
static pthread_t tid_socket;
static bool socket_has_data;

void *socket_thread(void *arg){
    while(1){
        while(client_fd == -1){
            // Wait for a connection
            client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_addr_len);
        }

        while(client_fd != -1){
            // Wait until there is data ready to be read
            struct pollfd pfd;
            pfd.fd = client_fd;
            pfd.events = POLLIN;
            int res = poll(&pfd, 1, -1);
            if(res == 0){
                // Timeout. Shouldn't happen here
                continue;
            }
            if((pfd.revents & POLLIN) != 0){
                // Have data
                socket_has_data = true;
                
                // Clear this event
                pfd.revents &= ~POLLIN;
            }

            if(pfd.revents != 0){
                // Any other events would be errors
                socket_has_data = false;
                close(client_fd);
                client_fd = -1;
            }
        }
    }
    return NULL;
}

static void usb_socket_cleanup(void){
    close(client_fd);
    close(server_fd);
}

bool usb_setup_socket(int port){
    // Ensure proper cleanup
    atexit(usb_socket_cleanup);

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
    socket_has_data = false;

    // Success
    return true;
}

void usb_sim_interrupts(void){
    if(socket_has_data){
        socket_has_data = false;

        unsigned int avail;
        if(ioctl(client_fd, FIONREAD, &avail) == -1){
            // Assume connection loss
            return;
        }
        if(avail == 0)
            return; // Shouldn't happen, but just in case

        uint8_t b;
        for(unsigned int i = 0; i < avail; ++i){
            if(read(client_fd, &b, 1) == -1){
                // Assume connection loss
                return;
            }
            // Write data into the read buffer
            if(!cb_write(&read_buf, b))
                break;
        }

        // Give the semaphore b/c there's now data in the read buffer
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(avail_to_read_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void usb_init(void){
    // Buffers setup
    write_buf_pos = 0;
    cb_init(&read_buf, read_buf_arr, USB_RB_SIZE);
    avail_to_read_sem = xSemaphoreCreateBinary();

    // Parallel threads are used for read / write from / to socket
    // This prevents unexpected blocking behavior with RTOS threads
    // Things go poorly when RTOS threads invoke syscalls (blocks entire program)
    // So generally, keep syscalls to parallel threads
    // These parallel threads need signals masked though to prevent issues with RTOS function
    // See: https://www.freertos.org/Documentation/api-ref/POSIX/index.html
    // Since pthread_create makes a child inheriting the parent's sigmask, mask all here before creation
    // This is a portable way of ensuring that created thread's sigmask is correct at creation
    // Then, after creation, restore mask here so this still works as expected by RTOS port.
    sigset_t fullset, origset;
    sigfillset(&fullset);
    pthread_sigmask(SIG_SETMASK, &fullset, &origset);
    pthread_create(&tid_socket, NULL, socket_thread, NULL);
    pthread_sigmask(SIG_SETMASK, &origset, NULL);

    usb_initialized = true;
}

void usb_process(void){
    xSemaphoreTake(avail_to_read_sem, portMAX_DELAY);
}

unsigned int usb_avail(void){
    return CB_AVAIL_READ(&read_buf);
}

uint8_t usb_read(void){
    uint8_t b;
    cb_read(&read_buf, &b);
    return b;
}

void usb_write(uint8_t b){
    write_buf[write_buf_pos] = b;
    write_buf_pos++;
    if(write_buf_pos == USB_WB_SIZE)
        usb_flush();
}

static void do_write(int, uint8_t*, unsigned int);

void usb_flush(void){
    do_write(client_fd, write_buf, write_buf_pos);
    write_buf_pos = 0;
}

static void do_write(int fd, uint8_t *buf, unsigned int count){
    if(client_fd == -1)
        return;
    
    // Have to mask SIGPIPE to prevent program termination when client disconnects
    // The (more recent) POSIX standard way to do this is MSG_NOSIGPIPE in send syscall
    // But macOS / *BSD seem to not support this (yet?). They use a sockoption, but that
    // doesn't work for write call only send?
    // And windows doesn't have SIGPIPE at all (one of the few cases windows is sane!)
    // So instead of trying to prevent SIGPIPE with sigaction (Liunx specific), signal 
    // (undefined behavior on Linux with multiple threads), etc just use pthreads masking
    sigset_t set_mask_pipe;
    sigemptyset(&set_mask_pipe);
    sigaddset(&set_mask_pipe, SIGPIPE);
    sigset_t old_set;
    pthread_sigmask(SIG_BLOCK, &set_mask_pipe, &old_set);
    
    ssize_t res = write(fd, buf, count);
    if(res == -1 && errno == EPIPE){
        // Other side disconnected
        socket_has_data = false;
        close(client_fd);
        client_fd = -1;
    }

    // After write call, have to handle the SIGPIPE if it is pending, otherwise it will be
    // Sent elsewhere later
    sigset_t set_pending;
    sigpending(&set_pending);
    if(sigismember(&set_pending, SIGPIPE)){
        // Got a SIGPIPE. Must handle it before unmasking!
        int res;
        sigwait(&set_mask_pipe, &res);
    }

    // Unmask
    pthread_sigmask(SIG_SETMASK, &old_set, NULL);
}

#endif // CONTROL_BOARD_SIM_LINUX || CONTROL_BOARD_SIM_MACOS


#if defined(CONTROL_BOARD_SIM_WIN)

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <util/circular_buffer.h>

#include <WinSock2.h>
#include <Windows.h>

// Buffers
// Write buffer is linear (it is always emptied in full)
// Read buffer is circular (ring) buffer
#define USB_WB_SIZE 128
#define USB_RB_SIZE 128
static uint8_t write_buf[USB_WB_SIZE];
static unsigned int write_buf_pos;
static uint8_t read_buf_arr[USB_RB_SIZE];
static circular_buffer read_buf;
static SemaphoreHandle_t avail_to_read_sem;

// Socket & thread stuff
static WSADATA wsa;
static SOCKET server_sock;
static SOCKET client_sock;
static bool socket_has_data;
static HANDLE sock_thread_handle;

DWORD WINAPI socket_thread(void *arg){
    while(1){
        while(client_sock == INVALID_SOCKET){
            // Wait for a connection
            client_sock = accept(server_sock, NULL, NULL);
        }

        while(client_sock != INVALID_SOCKET){
            // Wait until there is data ready to be read
            WSAPOLLFD pfd = {0};
            pfd.fd = client_sock;
            pfd.events = POLLIN;
            int res = WSAPoll(&pfd, 1, -1);
            if(res == 0){
                // Timeout. Shouldn't happen here
                continue;
            }
            if((pfd.revents & POLLIN) != 0){
                // Have data
                socket_has_data = true;
                
                // Clear this event
                pfd.revents &= ~POLLIN;
            }

            if(pfd.revents != 0){
                // Any other events would be errors
                socket_has_data = false;
                closesocket(client_sock);
                client_sock = INVALID_SOCKET;
            }
        }
    }
    return 0;
}

static void usb_socket_cleanup(void){
    closesocket(client_sock);
    closesocket(server_sock);
    WSACleanup();
}

bool usb_setup_socket(int port){
    // Ensure proper cleanup
    atexit(usb_socket_cleanup);

    // Initialize winsock2
    int err = WSAStartup(MAKEWORD(2,2), &wsa);
    if (err != 0){
        fprintf(stderr, "Failed to initialize winsock2! Error code: %d\n", err);
        return false;
    }

    // Setup socket
    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if(server_sock == INVALID_SOCKET){
        fprintf(stderr, "Failed to create socket. Error code: %d\n", WSAGetLastError());
        return false;
    }
    struct sockaddr_in a;
    memset(&a, '0', sizeof(a));
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = htons(port);
    if(bind(server_sock, (struct sockaddr *)&a, sizeof(a)) == SOCKET_ERROR){
        fprintf(stderr, "Failed to bind socket. Error code: %d\n", WSAGetLastError());
        return false;
    }
    if(listen(server_sock, 1) == SOCKET_ERROR){
        fprintf(stderr, "Failed to listen on socket. Error code: %d\n", WSAGetLastError());
        return false;
    }
    client_sock = INVALID_SOCKET;
    socket_has_data = false;

    // Success
    return true;
}

void usb_sim_interrupts(void){
    if(socket_has_data){
        socket_has_data = false;

        u_long avail;
        if(ioctlsocket(client_sock, FIONREAD, &avail) == SOCKET_ERROR){
            // Assume connection loss
            return;
        }
        if(avail == 0)
            return; // Shouldn't happen, but just in case

        uint8_t b;
        for(u_long i = 0; i < avail; ++i){
            if(recv(client_sock, &b, 1, 0) == SOCKET_ERROR){
                // Assume connection loss
                return;
            }
            // Write data into the read buffer
            if(!cb_write(&read_buf, b))
                break;
        }

        // Give the semaphore b/c there's now data in the read buffer
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(avail_to_read_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void usb_init(void){
    // Buffers setup
    write_buf_pos = 0;
    cb_init(&read_buf, read_buf_arr, USB_RB_SIZE);
    avail_to_read_sem = xSemaphoreCreateBinary();

    // Create thread for socket!
    sock_thread_handle = CreateThread(NULL, 0, socket_thread, NULL, 0, NULL);

    usb_initialized = true;
}

void usb_process(void){
    xSemaphoreTake(avail_to_read_sem, portMAX_DELAY);
}

unsigned int usb_avail(void){
    return CB_AVAIL_READ(&read_buf);
}

uint8_t usb_read(void){
    uint8_t b;
    cb_read(&read_buf, &b);
    return b;
}

void usb_write(uint8_t b){
    write_buf[write_buf_pos] = b;
    write_buf_pos++;
    if(write_buf_pos == USB_WB_SIZE)
        usb_flush();
}

static void do_write(int, uint8_t*, unsigned int);

void usb_flush(void){
    do_write(client_sock, write_buf, write_buf_pos);
    write_buf_pos = 0;
}

static void do_write(int fd, uint8_t *buf, unsigned int count){
    if(client_sock == INVALID_SOCKET)
        return;
    
    if(send(fd, buf, count, 0) == SOCKET_ERROR){
        // Other side disconnected
        socket_has_data = false;
        closesocket(client_sock);
        client_sock = INVALID_SOCKET;
    }
}

#endif // CONTROL_BOARD_SIM_WIN
