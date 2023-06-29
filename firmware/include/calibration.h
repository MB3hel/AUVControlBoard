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
    // Is data valid
    bool valid;

    // Accelerometer data (BNO055)
    int16_t accel_offset_x;
    int16_t accel_offset_y; 
    int16_t accel_offset_z; 
    int16_t accel_radius;

    // Gyro data (BNO055)
    int16_t gyro_offset_x;
    int16_t gyro_offset_y;
    int16_t gyro_offset_z;
} bno055_cal_t;

typedef struct {
    // Atmospheric pressure at surface
    float atm_pressure;

    // Fluid density
    float fluid_density;
} ms5837_cal_t;


extern bno055_cal_t calibration_bno055;
extern ms5837_cal_t calibration_ms5837;


/**
 * Load calibration constants for all sensors
 * 
 */
void calibration_load(void);

/**
 * Load calibrations constants for BNO055 from eeprom
 */
void calibration_load_bno055(void);

/**
 * Store calibration constants for BNO055 to eeprom
 */
void calibration_store_bno055(bno055_cal_t new_data);

/**
 * Erase calibration constants for BNO055 from eeprom
 */
void calibration_erase_bno055(void);

/**
 * Load calibrations constants for MS5837 from eeprom
 */
void calibration_load_ms5837(void);

/**
 * Store calibration constants for MS5837 to eeprom
 */
void calibration_store_ms5837(ms5837_cal_t new_data);

/**
 * Erase calibration constants for MS5837 from eeprom
 */
void calibration_erase_ms5837(void);
