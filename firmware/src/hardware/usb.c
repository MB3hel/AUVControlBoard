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

#define USB_WB_SIZE 256
#define USB_RB_SIZE 512
#define STDIN_BLOCK_SIZE 32

// Uses stdio to send data that would have been sent over USB by physical board
// Data is buffered here because IO at OS level can be expensive (depends on OS)
// Also, for reading data, need to be able to get num available bytes, which is
// generally not possible directly using stdio
// Write buffer is linear b/c it is always written out to stdio fully
// However read buffer is circular b/c usb_read reads one at a time
static uint8_t write_buf[USB_WB_SIZE];
static size_t write_buf_pos;
static uint8_t read_cb_arr[USB_RB_SIZE];
static circular_buffer read_cb;

// Directly read into this buffer before copy to CB
static uint8_t stdin_block[STDIN_BLOCK_SIZE];

// USB read and process occur on different threads
// Both can result in circular buffer's size changing, so protect with mutex
static SemaphoreHandle_t read_buf_mutex;

// Write buffer size may change from write and flush, which can (but typically are not)
// called from different threads
static SemaphoreHandle_t write_buf_mutex;

void usb_init(void){
    write_buf_pos = 0;
    cb_init(&read_cb, read_cb_arr, USB_RB_SIZE);

    read_buf_mutex = xSemaphoreCreateMutex();
    write_buf_mutex = xSemaphoreCreateMutex();
}

void usb_process(void){
    // fread waits for the input file (stdin) to not be empty
    // Then it reads count bytes (USB_PRB_SIZE in this case)
    // If the stream has fewer, it reaches EOF and return will be less
    size_t count = fread(stdin_block, sizeof(uint8_t), STDIN_BLOCK_SIZE, stdin);
    
    // Copy to actual read buffer
    xSemaphoreTake(read_buf_mutex, portMAX_DELAY);
    for(size_t i = 0; i < count; ++i){
        // Note that write will return false if buffer is full
        // But we just ignore that here (data discarded if buffer full)
        cb_write(&read_cb, stdin_block[i]);
    }
    xSemaphoreGive(read_buf_mutex);
}

unsigned int usb_avail(void){
    // Don't care about race when reading count, so no mutex
    return CB_AVAIL_READ(&read_cb);
}

uint8_t usb_read(void){
    uint8_t res = 0;
    // cb_read returns false and doesn't change res if buffer is empty
    cb_read(&read_cb, &res);
    return res;
}

void usb_write(uint8_t b){
    xSemaphoreTake(write_buf_mutex, portMAX_DELAY);
    write_buf[write_buf_pos] = b;
    write_buf_pos++;

    // If buffer is full, flush now
    if(write_buf_pos == USB_WB_SIZE){
        xSemaphoreGive(write_buf_mutex);
        usb_flush();
    }else{
        xSemaphoreGive(write_buf_mutex);
    }
}

void usb_flush(void){
    // Write whatever is currently in output buffer now
    xSemaphoreTake(write_buf_mutex, portMAX_DELAY);
    fwrite(write_buf, sizeof(uint8_t), write_buf_pos, stdout);
    write_buf_pos = 0;
    xSemaphoreGive(write_buf_mutex);
}

#endif // CONTROL_BOARD_SIM
