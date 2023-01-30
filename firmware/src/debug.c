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


void debug_halt(int error_code){
    taskDISABLE_INTERRUPTS();
    led_set(255, 0, 0);
    // TODO: Store error code in memory where it can be retrieved on next boot
    while(1){
        // Note: nop is here for debugger
        asm("nop");
    }
}

void debug_log(const char *msg){
#ifndef NDEBUG
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
    pccomm_write(buf, len);
#endif
}
