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

#include <framework.h>
#include <FreeRTOS.h>
#include <task.h>
#include <led.h>
#include <delay.h>
#include <math.h>
#include <usb.h>
#include <thruster.h>
#include <app.h>
#include <cmdctrl.h>
#include <conversions.h>
#include <motor_control.h>
#include <i2c.h>
#include <wdt.h>


#if defined(CONTROL_BOARD_V2)
extern RTC_HandleTypeDef hrtc;
#endif


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Program Entry point / startup
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(void){
    init_frameworks();

    // -------------------------------------------------------------------------
    // Bootloader check (v2 only)
    // -------------------------------------------------------------------------
#if defined(CONTROL_BOARD_V2)
    // Reboot to bootloader is implemented by writing backup register then resetting
    // See usb.c for details
    if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) == 0x3851FDEB){
        // Clear so next boot is normal
        HAL_PWR_EnableBkUpAccess();
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x12345678);
        HAL_PWR_DisableBkUpAccess();

        // Jump to bootloader
        HAL_SuspendTick();
        HAL_RCC_DeInit();
        HAL_DeInit();
        HAL_FLASH_Unlock();
        HAL_FLASH_OB_Unlock();
        __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
        __set_MSP(*((uint32_t*) 0x00000000));
        ((void (*)(void)) *((uint32_t*) 0x00000004))();
    }
#endif

    // -------------------------------------------------------------------------
    // System & Peripheral Initialization
    // -------------------------------------------------------------------------
    conversions_init();
    wdt_init();
    delay_init();
    led_init();
    usb_init();
    thruster_init();
    mc_init();
    cmdctrl_init();
    i2c_init();
    // -------------------------------------------------------------------------
    
    app_init();

    // -------------------------------------------------------------------------
    // RTOS Startup
    // -------------------------------------------------------------------------
    vTaskStartScheduler();
    // -------------------------------------------------------------------------

    // Start scheduler should never return. This should never run, but is
    // included to make debugging easier in case it does
    taskDISABLE_INTERRUPTS();
    led_set(255, 0, 0);
    while(1){
        asm("nop");
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
