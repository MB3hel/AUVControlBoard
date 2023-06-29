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

#include <stdbool.h>



typedef struct{
    float depth_m;
    float pressure_pa;
    float temperature_c;
} ms5837_data;



/**
 * Initialize MS5837 driver
 */
void ms5837_init(void);

/**
 * Configure the sensor
 * 
 * @return true on success; false on error
 */
bool ms5837_configure(void);

/**
 * Read data from sensor. Must be configured before running
 * 
 * @param data Pointer to struct to store data in (only valid if returns true)
 * @return true On success; false on error
 */
bool ms5837_read(ms5837_data *data);

