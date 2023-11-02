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
#include <hardware/delay.h>
#include <hardware/thruster.h>
#include <hardware/eeprom.h>
#include <hardware/i2c.h>
#include <hardware/wdt.h>

#include <util/conversions.h>

#include <framework.h>
#include <FreeRTOS.h>
#include <task.h>
#include <math.h>
#include <hardware/usb.h>
#include <app.h>
#include <cmdctrl.h>
#include <motor_control.h>
#include <debug.h>
#include <pccomm.h>
#include <calibration.h>


#ifdef CONTROL_BOARD_SIM
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#endif


// These variables persist across reset (used in framework.h for chip startup)
// Placed into section of RAM that is not zeroed on reset
// V2 doesn't need these because it uses the RTC backup registers instead
#if defined(CONTROL_BOARD_V1)
__attribute__((section(".noinit"))) volatile uint32_t first_run;
__attribute__((section(".noinit"))) volatile uint32_t reset_cause_persist;
#endif


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Program Entry point / startup
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifdef CONTROL_BOARD_SIM
int main(int argc, char** argv){
#else
int main(void){
#endif

#if defined(CONTROL_BOARD_SIM)
    if(argc != 2){
        fprintf(stderr, "Usage: %s [port]\n", argv[0]);
        return 1;
    }
    for(size_t i = 0; i < strlen(argv[1]); ++i){
        if(!isdigit(argv[1][i])){
            fprintf(stderr, "Invalid port number.\n");
            return 1;
        }
    }
    int port = atoi(argv[1]);
    if(port > 65535 || port < 0){
        fprintf(stderr, "Invalid port number.\n");
    }
    if(!usb_setup_socket(port)){
        return 1;
    }
#endif
    // -------------------------------------------------------------------------
    // System & Peripheral Initialization
    // -------------------------------------------------------------------------

    // Initialize HAL & Vendor libraries & perform startup checks
    init_frameworks();

    // Anything else may use conversions, so init before hardware
    conversions_init();

    // Init hardware
    delay_init();
    led_init();
    // usb_init();   Don't init usb yet. Do that after RTOS started (b/c TinyUSB uses RTOS stuff)
    thruster_init();
    i2c_init();
    eeprom_init();
#ifdef NDEBUG
    wdt_init();             // Watchdog disabled for DEBUG builds
#endif

    // Initialize communication, motor control, and cmdctrl
    pccomm_init();
    mc_init();
    cmdctrl_init();

    // Load calibration data before starting RTOS (must happen before sensors initalized in RTOS threads)
    calibration_load();
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
