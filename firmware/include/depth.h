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

#pragma once

#include <util/angles.h>
#include <stdbool.h>
#include <stdint.h>


#define DEPTH_NONE        0
#define DEPTH_SIM         1
#define DEPTH_MS5837      2


typedef struct{
    // All depth sensors provide this value.
    float depth_m;

    // The following raw measurements may not be supported by all sensors
    // Ex: If an acoustic depth sensor were used, pressure and temperature would
    //     not be provided.
    float pressure_pa;
    float temperature_c;
} depth_data_t;



/**
 * Initialize depth sensor(s)
 */
void depth_init(void);

/**
 * Get current data from IMU
 * @return true on success, false on failure
 */
bool depth_read(void);

/**
 * Get the current depth data. Thread safe.
 * @return the data
 */
depth_data_t depth_get_data(void);

/**
 * Get the currently active depth sensor. Thread safe.
 * @return uint8_t ID of the active sensor (DEPTH_xyz)
 */
uint8_t depth_get_sensor(void); 
