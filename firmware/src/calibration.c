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
#include <hardware/eeprom.h>
#include <util/conversions.h>

bno055_cal_t calibration_bno055;
ms5837_cal_t calibration_ms5837;


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
    calibration_load_ms5837();
}

void calibration_load_bno055(void){
    uint16_t sig = 0;
    eeprom_read(BNO055_SIG_IDX, &sig);
    calibration_bno055.valid = (sig == SIG_VALID);
    if(calibration_bno055.valid){
        eeprom_read(BNO055_ACC_X_IDX, (uint16_t*)&calibration_bno055.accel_offset_x);
        eeprom_read(BNO055_ACC_Y_IDX, (uint16_t*)&calibration_bno055.accel_offset_y);
        eeprom_read(BNO055_ACC_Z_IDX, (uint16_t*)&calibration_bno055.accel_offset_z);
        eeprom_read(BNO055_ACC_R_IDX, (uint16_t*)&calibration_bno055.accel_radius);
        eeprom_read(BNO055_GYR_X_IDX, (uint16_t*)&calibration_bno055.gyro_offset_x);
        eeprom_read(BNO055_GYR_Y_IDX, (uint16_t*)&calibration_bno055.gyro_offset_y);
        eeprom_read(BNO055_GYR_Z_IDX, (uint16_t*)&calibration_bno055.gyro_offset_z);
    }
}

void calibration_store_bno055(bno055_cal_t new_data){
    eeprom_write(BNO055_ACC_X_IDX, (uint16_t)new_data.accel_offset_x);
    eeprom_write(BNO055_ACC_Y_IDX, (uint16_t)new_data.accel_offset_y);
    eeprom_write(BNO055_ACC_Z_IDX, (uint16_t)new_data.accel_offset_z);
    eeprom_write(BNO055_ACC_R_IDX, (uint16_t)new_data.accel_radius);
    eeprom_write(BNO055_GYR_X_IDX, (uint16_t)new_data.gyro_offset_x);
    eeprom_write(BNO055_GYR_Y_IDX, (uint16_t)new_data.gyro_offset_y);
    eeprom_write(BNO055_GYR_Z_IDX, (uint16_t)new_data.gyro_offset_z);
    eeprom_write(BNO055_SIG_IDX, SIG_VALID);
    calibration_bno055 = new_data;
    calibration_bno055.valid = true;
}

void calibration_erase_bno055(void){
    // Instead of actually erasing data (potentially more flash writes)
    // Just invalidate the signature (fewer flash writes / erases)
    eeprom_write(BNO055_SIG_IDX, 0x0001);
    calibration_bno055.valid = false;
}

void calibration_load_ms5837(void){
    // MS5837 calibration is not persistent, thus no use of eeprom
    // Making it persistent would not make sense
    // - It will change almost every time the vehicle is used (atm_pressure)
    // - It is easy to calibrate on-vehicle at vehicle startup (just read the pressure!)
    // Making it persistent would also result in many write to eeprom (flash emulated) which
    // could wear things out faster than desired.
    // Thus, just assign defaults here
    calibration_ms5837.atm_pressure = 101325.0f;
    calibration_ms5837.fluid_density = 997.0f;
}

