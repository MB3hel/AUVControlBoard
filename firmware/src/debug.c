/*
 * Copyright 2023 Marcus Behel
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

#include <debug.h>
#include <pccomm.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <led.h>
#include <usb.h>
#include <stdlib.h>


void debug_halt(int error_code){
    (void)error_code;

    taskDISABLE_INTERRUPTS();
    led_set(255, 0, 0);
    // TODO: Store error code in memory where it can be retrieved on next boot
    char *msg = "HALT:      ";
    itoa(error_code, &msg[6], 10);
    pccomm_write((uint8_t*)msg, 11);
    while(1){
        // Note: nop is here for debugger
        asm("nop");
    }
}

void debug_log(const char *msg){
#ifndef NDEBUG
    if(!usb_initialized)
        return;
    if(xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
        return;
    
    // Only enable logging for debug builds
    uint8_t buf[PCCOMM_MAX_MSG_LEN];
    buf[0] = 'D';
    buf[1] = 'E';
    buf[2] = 'B';
    buf[3] = 'U';
    buf[4] = 'G';
    unsigned int len = strlen(msg);
    if(len > (PCCOMM_MAX_MSG_LEN - 5))
        len = PCCOMM_MAX_MSG_LEN - 5;
    for(unsigned int i = 0; i < len; ++i)
        buf[5+i] = msg[i];
    pccomm_write(buf, len + 5);
#else
    (void)msg;
#endif
}

void debug_log_data(uint8_t *msg, unsigned int len){
#ifndef NDEBUG
    if(!usb_initialized)
        return;
    if(xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
        return;
    
    // Only enable logging for debug builds
    uint8_t buf[PCCOMM_MAX_MSG_LEN];
    buf[0] = 'D';
    buf[1] = 'B';
    buf[2] = 'G';
    buf[3] = 'D';
    buf[4] = 'A';
    buf[5] = 'T';
    if(len > (PCCOMM_MAX_MSG_LEN - 6))
        len = PCCOMM_MAX_MSG_LEN - 6;
    for(unsigned int i = 0; i < len; ++i)
        buf[6+i] = msg[i];
    pccomm_write(buf, len + 6);
#else
    (void)msg;
    (void)len;
#endif
}
