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

#pragma once

#include <stdint.h>

#if defined(CONTROL_BOARD_V1)

extern uint32_t SystemCoreClock;

#include <samd51g19a.h>
#include <definitions.h>
#include <device.h>

static inline __attribute__((always_inline)) void init_frameworks(void){
    // Enable FPU
    SCB->CPACR |= (3UL << 10*2) | (3UL << 11*2);

    // Initialize MCC generated code
    SYS_Initialize(NULL);
}

#elif defined(CONTROL_BOARD_V2)

#include <stm32f4xx.h>
#include <stm32f4xx_hal.h>
#include <stm32cubemx_main.h>

extern void stm32cubemx_main(void);

static inline __attribute__((always_inline)) void init_frameworks(void){
    // Enable FPU
    SCB->CPACR |= (3UL << 10*2) | (3UL << 11*2);

    // Initialize STM32CubeMX generated code
    stm32cubemx_main();
}

#endif