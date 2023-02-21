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
    debug_halt(HALT_EC_SCHEDRET);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
