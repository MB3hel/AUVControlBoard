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
#include <stdint.h>
#include <imu.h>
#include <util/angles.h>


// Axis configurations
// Note: Px macro needs to have value x
#define BNO055_AXIS_P0          0
#define BNO055_AXIS_P1          1
#define BNO055_AXIS_P2          2
#define BNO055_AXIS_P3          3
#define BNO055_AXIS_P4          4
#define BNO055_AXIS_P5          5
#define BNO055_AXIS_P6          6
#define BNO055_AXIS_P7          7


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// BNO055 Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Initialize BNO055 driver
 */
void bno055_init(void);

/**
 * Configure the sensor
 * 
 * @return true on success; false on error
 */
bool bno055_configure(void);

/**
 * Set axis remap and sign configuration. Must be configured before running
 * 
 * @param mode Mode to set (BNO055_AXIS_Px)
 * @return true on success; false on error
 */
bool bno055_set_axis(uint8_t mode);


/**
 * Read data from IMU. Must be configured before running
 * 
 * @param data imu_data struct to read data into (does not calc accum angles)
 * @return true On success; false on error
 */
bool bno055_read(imu_data_t *data);

/**
 * Get calibration status value from BNO055
 * @param status Where to store status byte
 * @return true On read success
 * @return false On read failure
 */
bool bno055_read_calibration_status(uint8_t *status);

/**
 * 
 * Get calibration values for the sensor (only works properly after calibration status is valid)
 * 
 * @param status Where to store CALIB_STAT register value
 * @return true on success; false on read failure
 */
bool bno055_read_calibration(int16_t *acc_offset_x, int16_t *acc_offset_y, int16_t *acc_offset_z, 
        int16_t *acc_radius, int16_t *gyr_offset_x, int16_t *gyr_offset_y, int16_t *gyr_offset_z);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

