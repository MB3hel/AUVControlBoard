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

#include <stdbool.h>
#include <stdint.h>


typedef struct {
    // Accelerometer data (BNO055)
    uint16_t accel_offset_x;
    uint16_t accel_offset_y; 
    uint16_t accel_offset_z; 
    uint16_t accel_radius;

    // Gyro data (BNO055)
    uint16_t gyro_offset_x;
    uint16_t gyro_offset_y;
    uint16_t gyro_offset_z;
} calibration_data_t;


extern bool calibration_valid;
extern calibration_data_t calibration_data;


/**
 * Load calibrations from eeprom
 */
void calibration_load(void);

/**
 * Store the given calibration to eeprom
 */
void calibration_store(calibration_data_t new_data);

/**
 * Erase stored calibration data (invalidates it)
 */
void calibration_erase(void);
