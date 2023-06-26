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


static const uint16_t cal_sig[4] = {0x3B3B, 0x3B3B};


bool calibration_valid = false;
calibration_data_t calibration_data;


// Indices (addresses) of different values (16-bit) in eeprom
#define SIG_IDX     (0x0000)        // First 2 values are signature
//                  (0x0001)        // Also signature
#define ACC_X_IDX   (0x0002)        // Accel offset x
#define ACC_Y_IDX   (0x0003)        // Accel offset y
#define ACC_Z_IDX   (0x0004)        // Accel offset z
#define ACC_R_IDX   (0x0005)        // Accel radius
#define GYR_X_IDX   (0x0006)        // Gyro offset x
#define GYR_Y_IDX   (0x0007)        // Gyro offset y
#define GYR_Z_IDX   (0x0008)        // Gyro offset z



void calibration_load(void){
    // First 2 half-words are a calibration signature. If the bytes match this signature,
    // it is assumed that a valid set of calibration constants are stored in the eeprom.
    // Otherwise, it is assumed that the calibration data is not valid (has never been stored).

    // This is the correct signature. Will be compared to read signature bytes
    

    // Read signature from eeprom and compare
    uint16_t val;
    bool valid = true;
    for(unsigned int i = 0; i < 2; ++i){
        eeprom_read(SIG_IDX + i, &val);
        if(val != cal_sig[i]){
            valid = false;
            break;
        }
    }
    if(valid){
        eeprom_read(ACC_X_IDX, &calibration_data.accel_offset_x);
        eeprom_read(ACC_Y_IDX, &calibration_data.accel_offset_y);
        eeprom_read(ACC_Z_IDX, &calibration_data.accel_offset_z);
        eeprom_read(ACC_R_IDX, &calibration_data.accel_radius);
        eeprom_read(GYR_X_IDX, &calibration_data.gyro_offset_x);
        eeprom_read(GYR_Y_IDX, &calibration_data.gyro_offset_y);
        eeprom_read(GYR_Z_IDX, &calibration_data.gyro_offset_z);
    }
    calibration_valid = valid;
}

void calibration_store(calibration_data_t new_data){
    eeprom_write(ACC_X_IDX, new_data.accel_offset_x);
    eeprom_write(ACC_Y_IDX, new_data.accel_offset_x);
    eeprom_write(ACC_Z_IDX, new_data.accel_offset_x);
    eeprom_write(ACC_R_IDX, new_data.accel_radius);
    eeprom_write(GYR_X_IDX, new_data.gyro_offset_x);
    eeprom_write(GYR_Y_IDX, new_data.gyro_offset_y);
    eeprom_write(GYR_Z_IDX, new_data.gyro_offset_z);
    eeprom_write(SIG_IDX, cal_sig[0]);
    eeprom_write(SIG_IDX + 1, cal_sig[0]);
    calibration_data = new_data;
}

void calibration_erase(void){
    // Instead of actually erasing data (potentially more flash writes)
    // Just invalidate the signature (fewer flash writes)
    eeprom_write(SIG_IDX, 0xFF);
}
