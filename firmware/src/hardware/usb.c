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
#include <hardware/delay.h>

#ifdef CONTROL_BOARD_SIM_LINUX
#include <signal.h>
#include <pthread.h>
#endif

#define USB_WB_SIZE 64
#define USB_RB_SIZE 128

// Write buffer
// No need for circular buffer b/c always written out in whole
static uint8_t write_buf[USB_WB_SIZE];
static unsigned int write_buf_pos;

// Read buffer
static uint8_t read_cb_arr[USB_RB_SIZE];
static circular_buffer read_cb;

// This RTOS semaphore is given by STDIN thread and taken by usb_process
// Can use binary semaphore in STDIN thread b/c the FromISR functions support
// binary semaphore
static SemaphoreHandle_t read_ready_sem;

// Both buffers must be mutex protected as different threads may both change
// the count / position variables
static SemaphoreHandle_t write_mutex;

// Read mutex is taken by non-RTOS thread, so using pthread mutex
// RTOS thread can't context switch when it's blocking on this
// But that's OK in this context. Similar to what an ISR would end
// up causing.
pthread_mutex_t read_mutex; 

static void *stdin_handler(void *arg){
    // STDIN thread
    // This is a non-RTOS thread that runs in parallel to RTOS threads
    // Handles STDIN and "simulates interrupts" for USB data handling
    // To then give mutex

    uint8_t b;
    size_t count;
    while(true){
        // This function blocks until there is some data to read
        count = fread(&b, sizeof(uint8_t), 1, stdin);
        if(count != 0){
            // Copy the data into the read buffer
            pthread_mutex_lock(&read_mutex);
            // Note that if read_cb is full cb_write function will
            // not overwrite array but return false
            // If buffer full, data is discarded
            if(!cb_write(&read_cb, b))
                break;
            pthread_mutex_unlock(&read_mutex);
        }


        // Give the semaphore signaling that there is data in the buffer
        BaseType_t xHigherPriorityTaskWoken;
        xSemaphoreGiveFromISR(read_ready_sem, &xHigherPriorityTaskWoken);
        // Do NOT actually portYIELD_FROM_ISR here! This is not a real ISR
        // nor is it running in memory context of an RTOS thread!
    }
    return NULL;
}

void usb_init(void){
    write_buf_pos = 0;
    cb_init(&read_cb, read_cb_arr, USB_RB_SIZE);

    // Semaphore count is initially 0 on creation (must give before take)
    read_ready_sem = xSemaphoreCreateBinary();

    // Mutexes are created ready to be taken
    write_mutex = xSemaphoreCreateMutex();
    pthread_mutex_init(&read_mutex, NULL);

    // Create STDIN thread
    // Reading STDIN blocks and blocking on an RTOS thread can hang entire process
    // Thus use a parallel thread "simulating interrupts"
    // This thread has signals masked b/c the RTOS port uses signals to context
    // switch and this is not a thread involved in context switching.
    // Note that to ensure correct operation, must mask all signals when the thread
    // is created. GNU extension allows doing this with pthread_attr_t and
    // pthread_attr_setsigmask_np, but this is not portable to other systems
    // So, instead, mask all signals here before create because the crated
    // thread inherits a copy of the creating thead's signal mask
    // After creation, the original mask is restored for this thread
    sigset_t fullset, origset;
    sigfillset(&fullset);
    pthread_sigmask(SIG_SETMASK, &fullset, &origset);
    pthread_t tid;
    pthread_create(&tid, NULL, stdin_handler, NULL);
    pthread_sigmask(SIG_SETMASK, &origset, NULL);

    usb_initialized = true;
}

void usb_process(void){
    while(xSemaphoreTake(read_ready_sem, portMAX_DELAY) == pdFALSE);
    // Will return once there's data in the read buffer (which is once signaled)
}

unsigned int usb_avail(void){
    // No mutex needed here b/c only reading the count, not modifying it
    return CB_AVAIL_READ(&read_cb);
}

uint8_t usb_read(void){
    uint8_t b;
    pthread_mutex_lock(&read_mutex);
    // cb_read handles empty buffer (returns false)
    // In this case, b will never be assigned, but no memory issues
    cb_read(&read_cb, &b);
    pthread_mutex_unlock(&read_mutex);
    return b;
}

void usb_write(uint8_t b){
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
    // Writing to STDOUT will block, but not for long enough that it matters
    // Not really any longer than writing out over USB would take on real hardware
    xSemaphoreTake(write_mutex, portMAX_DELAY);
    if(write_buf_pos > 0){
        // It is recommended in the docs for the POSIX port to 
        // mask signals before these sort of calls (blocking OS calls like fwrite)
        // https://www.freertos.org/FreeRTOS-simulator-for-Linux.html
        sigset_t fullset, origset;
        sigfillset(&fullset);
        pthread_sigmask(SIG_SETMASK, &fullset, &origset);
        fwrite(write_buf, sizeof(uint8_t), write_buf_pos, stdout);
        fflush(stdout);
        pthread_sigmask(SIG_SETMASK, &origset, NULL);
        write_buf_pos = 0;
    }
    xSemaphoreGive(write_mutex);
}

#endif // CONTROL_BOARD_SIM
