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

#include <hardware/delay.h>
#include <framework.h>

#if defined(CONTROL_BOARD_V1) || defined(CONTROL_BOARD_V2)

void delay_init(void){
    // Blocking delays implemented using DWT
    // Note: disable then enable seems to be required on STM32
    //       Since it won't hurt elsewhere, just do it everywhere
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;        // Disable TCR
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;         // Enable TCR
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;                   // Disable clock cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;                    // Enable clock cycle counter
    DWT->CYCCNT = 0;                                        // Reset counter on enable
}

void delay_us(unsigned int us){
    uint32_t start = DWT->CYCCNT;
    us *= (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < us);
}

void delay_ms(unsigned int ms){
    uint32_t start = DWT->CYCCNT;
    ms *= (SystemCoreClock / 1000);
    while ((DWT->CYCCNT - start) < ms);
}

#endif // CONTROL_BOARD_V1 || CONTROL_BOARD_V2

#if defined(CONTROL_BOARD_SIM_LINUX)

#include <unistd.h>

// SimCB
void delay_init(void){}

void delay_us(unsigned int us){
    usleep(us);
}

void delay_ms(unsigned int ms){
    usleep(ms * 1000);
}

#endif // CONTROL_BOARD_SIM_LINUX
