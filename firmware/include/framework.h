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