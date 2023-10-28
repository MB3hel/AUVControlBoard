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
#include <debug.h>

#ifdef CONTROL_BOARD_V1

extern uint32_t SystemCoreClock;

#include <samd51g19a.h>
#include <definitions.h>
#include <device.h>

// Defined in main.c
extern __attribute__((section(".noinit"))) volatile uint32_t first_run;
extern __attribute__((section(".noinit"))) volatile uint32_t reset_cause_persist;

#define NOT_FIRST_RUN       0x1FEA7496

static inline __attribute__((always_inline)) void init_frameworks(void){
    // Enable FPU
    SCB->CPACR |= (3UL << 10*2) | (3UL << 11*2);

    // Initialize MCC generated code
    SYS_Initialize(NULL);

    if(first_run == NOT_FIRST_RUN){
        // This is not the first boot.
        // A soft reset occurred. Retrieve reason why.
        reset_cause = reset_cause_persist;
        reset_cause_persist = 0x00000000;

        // If reset cause is zero, check to see if it was watchdog triggered
        // This would indicate watchdog reset due to user code
        // Note that this omits watchdog reset from infinite loop in debug_halt
        // as this will write a different error code instead.
        if((reset_cause == 0) && (RSTC_REGS->RSTC_RCAUSE & RSTC_RCAUSE_WDT_Msk)){
            reset_cause = HALT_EC_WDOG;
        }
    }else{
        // First boot (hard reset)
        first_run = NOT_FIRST_RUN;
        reset_cause = 0x00000000;
        reset_cause_persist = 0x00000000;
    }
}

#endif // CONTROL_BOARD_V1


#ifdef CONTROL_BOARD_V2

#include <stm32f4xx.h>
#include <stm32f4xx_hal.h>
#include <stm32cubemx_main.h>
#include <stm32f4xx_hal_rcc.h>

#define BKUP_REG_BOOTLOADER         RTC_BKP_DR0             // Used to indicate should reboot to bootloader
#define BKUP_REG_RESETCAUSE         RTC_BKP_DR1             // Indicates last reboot cause

extern RTC_HandleTypeDef hrtc;

extern void stm32cubemx_main(void);

static inline __attribute__((always_inline)) void init_frameworks(void){
    // Enable FPU
    SCB->CPACR |= (3UL << 10*2) | (3UL << 11*2);

    // Initialize STM32CubeMX generated code
    stm32cubemx_main();

    // Reboot to bootloader is implemented by writing backup register then resetting
    // See usb.c for details
    if(HAL_RTCEx_BKUPRead(&hrtc, BKUP_REG_BOOTLOADER) == 0x3851FDEB){
        // Clear so next boot is normal
        HAL_PWR_EnableBkUpAccess();
        HAL_RTCEx_BKUPWrite(&hrtc, BKUP_REG_BOOTLOADER, 0x00000000);
        HAL_PWR_DisableBkUpAccess();

        // Jump to bootloader
        // Adapted from
        // - https://michaeltien8901.github.io/2020/05/30/Jump-to-Internal-Bootloader-STM32F4.html
        // Note: 0x1FFF0000 from STM32 Application Note AN2606
        void (*SysMemBootJump)(void);
        volatile uint32_t addr = 0x1FFF0000;
        HAL_SuspendTick();                                  // Disable HAL SysTick
        for (int i = 0; i < 8; i++){
            NVIC->ICER[i]=0xFFFFFFFF;                       // Clear interrupt enable
            NVIC->ICPR[i]=0xFFFFFFFF;                       // Clear interrupt pending
        }
        HAL_RCC_DeInit();                                   // Cleanup RCC
        HAL_DeInit();                                       // Cleanup HAL
        SysTick->CTRL = 0;                                  // Disable SysTick
        SysTick->LOAD = 0;                                  // Default Reload value
        SysTick->VAL = 0;                                   // Default value
        HAL_FLASH_Unlock();                                 // Allow FLASH control register access
        HAL_FLASH_OB_Unlock();                              // Allow flash option register access
        __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();             // Memory remap
        SysMemBootJump = (void (*)(void)) (*((uint32_t *)(addr + 4)));
        __set_MSP(*(uint32_t *)addr);                       // Set main stack pointer to 1 word before bootloader
        SysMemBootJump();                                   // Execute (jump to) bootloader
        while(1);                                           // Trap here if failure. WDT will "fix" if needed.
    }

    // Read last reboot error code & clear it
    reset_cause = HAL_RTCEx_BKUPRead(&hrtc, BKUP_REG_RESETCAUSE);
    HAL_PWR_EnableBkUpAccess();
    HAL_RTCEx_BKUPWrite(&hrtc, BKUP_REG_RESETCAUSE, 0x00000000);
    HAL_PWR_DisableBkUpAccess();

    // If reset cause is zero, check to see if it was watchdog triggered
    // This would indicate watchdog reset due to user code
    // Note that this omits watchdog reset from infinite loop in debug_halt
    // as this will write a different error code instead.
    if((reset_cause == 0) && __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)){
        reset_cause = HALT_EC_WDOG;
    }
    __HAL_RCC_CLEAR_RESET_FLAGS();
}
#endif // CONTROL_BOARD_V2


#ifdef CONTROL_BOARD_SIM
static inline __attribute__((always_inline)) void init_frameworks(void){}
#endif // CONTROL_BOARD_SIM