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

#include <hardware/led.h>
#include <framework.h>


#ifdef CONTROL_BOARD_V1

#include <hardware/delay.h>
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

#endif // CONTROL_BOARD_V1


#ifdef CONTROL_BOARD_V2

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

void led_init(void){
    // Configured in generated code to a PWM frequency of ~500Hz (96MHz / 3 / 65535 = 488MHz)
    // 16-bit resolution PWM (0-65535)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);           // R
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);           // G
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);           // B

    led_set(0, 0, 0);
}


void led_set(uint8_t r, uint8_t g, uint8_t b){
    // 255 - [r,g,b] because LED is common anode
    // Thus, lowest duty cycle is max brightness
    // Multiply by 255 so 0 -> 0 and 255 -> 65535
    // This timer is 16-bit but colors specified as 8-bit

    TIM1->CCR1 = (255 - r) * 257;
    TIM4->CCR1 = (255 - g) * 257;
    TIM4->CCR2 = (255 - b) * 257;
}

#endif // CONTROL_BOARD_V2
