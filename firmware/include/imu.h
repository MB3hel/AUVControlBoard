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


#define IMU_NONE        0
#define IMU_SIM         1
#define IMU_BNO055      2



typedef struct{
    float x, y, z;
} accel_data_t;

typedef struct{
    float x, y, z;
} gyro_data_t;

typedef struct{
    // All IMUs provide quaternion data and accumulated euler angles
    quaternion_t quat;
    euler_t accum_angles;

    // Raw gyro and accel data may not be provided by all IMUs (eg sim IMU doesn't provide)
    gyro_data_t raw_gyro;
    accel_data_t raw_accel;
} imu_data_t;



/**
 * Initialize IMU sensor(s)
 */
void imu_init(void);

/**
 * Get current data from IMU
 * @return true on success, false on failure
 */
bool imu_read(void);

/**
 * Get the current IMU data. Thread safe.
 * @return the data
 */
imu_data_t imu_get_data(void);

/**
 * Reset old IMU data (really only matters for accumulated angles)
 * Should be called by IMU drivers when the sensor's definition of axes changes.
 */
void imu_reset_data(void);

/**
 * Get the currently active IMU sensor. Thread safe.
 * @return uint8_t ID of the active sensor (IMU_xyz)
 */
uint8_t imu_get_sensor(void); 
