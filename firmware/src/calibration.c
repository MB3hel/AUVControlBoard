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


#include <calibration.h>
#include <stdint.h>
#include <eeprom.h>

bno055_cal_t calibration_bno055;


#define SIG_VALID 0x3B3B

// -----------------------------------------------------------------------------
// Indices (addresses) of different values (16-bit) in eeprom
// -----------------------------------------------------------------------------
// BNO055 IMU
#define BNO055_SIG_IDX      (0x0000)        // Valid calibration signature
#define BNO055_ACC_X_IDX    (0x0001)        // Accel offset x
#define BNO055_ACC_Y_IDX    (0x0002)        // Accel offset y
#define BNO055_ACC_Z_IDX    (0x0003)        // Accel offset z
#define BNO055_ACC_R_IDX    (0x0004)        // Accel radius
#define BNO055_GYR_X_IDX    (0x0005)        // Gyro offset x
#define BNO055_GYR_Y_IDX    (0x0006)        // Gyro offset y
#define BNO055_GYR_Z_IDX    (0x0007)        // Gyro offset z
// -----------------------------------------------------------------------------

void calibration_load(void){
    calibration_load_bno055();
}

void calibration_load_bno055(void){
    uint16_t sig;
    eeprom_read(BNO055_SIG_IDX, &sig);
    calibration_bno055.valid = (sig == SIG_VALID);
    if(calibration_bno055.valid){
        eeprom_read(BNO055_ACC_X_IDX, &calibration_bno055.accel_offset_x);
        eeprom_read(BNO055_ACC_Y_IDX, &calibration_bno055.accel_offset_y);
        eeprom_read(BNO055_ACC_Z_IDX, &calibration_bno055.accel_offset_z);
        eeprom_read(BNO055_ACC_R_IDX, &calibration_bno055.accel_radius);
        eeprom_read(BNO055_GYR_X_IDX, &calibration_bno055.gyro_offset_x);
        eeprom_read(BNO055_GYR_Y_IDX, &calibration_bno055.gyro_offset_y);
        eeprom_read(BNO055_GYR_Z_IDX, &calibration_bno055.gyro_offset_z);
    }
}

void calibration_store_bno055(bno055_cal_t new_data){
    eeprom_write(BNO055_ACC_X_IDX, new_data.accel_offset_x);
    eeprom_write(BNO055_ACC_Y_IDX, new_data.accel_offset_x);
    eeprom_write(BNO055_ACC_Z_IDX, new_data.accel_offset_x);
    eeprom_write(BNO055_ACC_R_IDX, new_data.accel_radius);
    eeprom_write(BNO055_GYR_X_IDX, new_data.gyro_offset_x);
    eeprom_write(BNO055_GYR_Y_IDX, new_data.gyro_offset_y);
    eeprom_write(BNO055_GYR_Z_IDX, new_data.gyro_offset_z);
    eeprom_write(BNO055_SIG_IDX, SIG_VALID);
    calibration_bno055 = new_data;
    calibration_bno055.valid = true;
}

void calibration_erase_bno055(void){
    // Instead of actually erasing data (potentially more flash writes)
    // Just invalidate the signature (fewer flash writes / erases)
    eeprom_write(BNO055_SIG_IDX, 0x0000);
    calibration_bno055.valid = false;
}
