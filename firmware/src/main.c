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
#include <debug.h>
#include <pccomm.h>
#include <eeprom.h>
#include <calibration.h>


#if defined(CONTROL_BOARD_V1)
__attribute__((section(".noinit"))) volatile uint32_t first_run;
__attribute__((section(".noinit"))) volatile uint32_t reset_cause_persist;
#endif


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Program Entry point / startup
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(void){
    // -------------------------------------------------------------------------
    // System & Peripheral Initialization
    // -------------------------------------------------------------------------
    init_frameworks();
    conversions_init();

#ifdef NDEBUG
    // Enable watchdog if not debug build
    wdt_init();
#endif

    delay_init();
    led_init();
    usb_init();
    pccomm_init();
    thruster_init();
    mc_init();
    cmdctrl_init();
    i2c_init();
    eeprom_init();
    calibration_load();

    uint16_t data;
    eeprom_read(0, &data);
    if(data == 0x2154){
        led_set(0, 255, 0);
        while(1);
    }else{
        bool res = eeprom_write(0, 0x2154);
        if(res)
            led_set(0, 0, 255);
        else
            led_set(255, 0, 0);
        while(1);
    }

    led_set(0, 0, 255);
    while(1);

    // -------------------------------------------------------------------------
    
    app_init();

    // -------------------------------------------------------------------------
    // RTOS Startup
    // -------------------------------------------------------------------------
    vTaskStartScheduler();
    // -------------------------------------------------------------------------

    // Start scheduler should never return. This should never run, but is
    // included to make debugging easier in case it does
    debug_halt(HALT_EC_SCHEDRET);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
