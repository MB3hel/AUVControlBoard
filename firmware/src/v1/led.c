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

#include <led.h>
#include <framework.h>
#include <delay.h>
#include <FreeRTOS.h>
#include <task.h>


void led_init(void){
    // All initialization handled in generated code
    led_set(0, 0, 0);
}

static inline void dotstar_write(uint8_t val){
    for (uint8_t bit = 8; bit > 0; bit--){
        if((val & (1 << (bit - 1))))
            DS_DAT_Set();
        else
            DS_DAT_Clear();
        DS_CLK_Set();
        delay_us(1);
        DS_CLK_Clear();
        delay_us(1);
    }
}

void led_set(uint8_t r, uint8_t g, uint8_t b){
    // ItsyBitsy M4 includes an RGB "dotstar" LED
    // The pins used for this are setup as gpio outputs in the MCC project
    // NOTE: Since this is bitbanged and requires fairly precise us level delays,
    //       this must disallow context switches while running
    taskENTER_CRITICAL();
    dotstar_write(0x00);
    dotstar_write(0x00);
    dotstar_write(0x00);
    dotstar_write(0x00);
    dotstar_write(0xFF);
    dotstar_write(b);
    dotstar_write(g);
    dotstar_write(r);
    dotstar_write(0xFF);
    taskEXIT_CRITICAL();
}
