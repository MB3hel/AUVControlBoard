/*
 * Copyright 2023 Marcus Behel
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

#include <depth.h>
#include <sensor/ms5837.h>
#include <cmdctrl.h>
#include <FreeRTOS.h>
#include <semphr.h>


static uint8_t depth_which;
static depth_data_t depth_data;
static depth_data_t new_data;
static unsigned int read_failures;
SemaphoreHandle_t depth_mutex;


void depth_init(void){
    // Default / initial values
    read_failures = 0;
    depth_data.depth_m = 0;
    depth_data.pressure_pa = 0;
    depth_data.temperature_c = 0;

    // Reading depth_data is multiple read operations
    // Want to ensure a write of depth_data cannot interrupt a read causing mixed data
    depth_mutex = xSemaphoreCreateMutex();

    // Init code for all supported depth sensors
    ms5837_init();
}


static void depth_configure_if_needed(void){
    // If sim hijacked, always use sim depth sensor
    if(cmdctrl_sim_hijacked){
        depth_which = DEPTH_SIM;
        read_failures = 0;
    }

    // If no longer sim hijacked, cannot use sim depth sensor
    if(!cmdctrl_sim_hijacked && depth_which == DEPTH_SIM){
        depth_which = DEPTH_NONE;
        read_failures = 0;
    }

    // If too many read failures, assume depth sensor is no longer connected
    if(read_failures >= 5){
        depth_which = DEPTH_NONE;
        read_failures = 0;
    }

    // Already an active depth sensor (no need to configure one)
    if(depth_which != DEPTH_NONE){
        return;
    }

    // Try to configure each depth sensor until one succeeds
    if(ms5837_configure()){
        depth_which = DEPTH_MS5837;
        return;
    }else{
        // Failed to configure all depth sensors
        depth_which = DEPTH_NONE;
    }
}

bool depth_read(void){

    // Configure depth sensor if needed
    depth_configure_if_needed();

    // If still no depth sensor, abort read
    if(depth_which == DEPTH_NONE){
        return false;
    }

    // Read the active depth sensor
    bool success = false;
    switch(depth_which){
    case DEPTH_SIM:
        // Read SIM depth sensor
        new_data.depth_m = cmdctrl_sim_depth;
        new_data.pressure_pa = 0;
        new_data.temperature_c = 0;
        success = true;
        break;
    case DEPTH_MS5837:
        success = ms5837_read(&new_data);
        break;
    }

    if(success){
        // Update depth_data while holding mutex
        xSemaphoreTake(depth_mutex, portMAX_DELAY);
        depth_data = new_data;
        xSemaphoreGive(depth_mutex);

    }else{
        read_failures++;
    }

    return success;
}

depth_data_t depth_get_data(void){
    depth_data_t ret_data;

    // Read under mutex because reading a struct is multiple reads
    // If reading from one thread and writing depth_data (via depth_read() function) on another thread
    // depth_data could change part way through the read here.
    xSemaphoreTake(depth_mutex, portMAX_DELAY);
    ret_data = depth_data;
    xSemaphoreGive(depth_mutex);

    return ret_data;
}

uint8_t depth_get_sensor(void){
    return depth_which;
}