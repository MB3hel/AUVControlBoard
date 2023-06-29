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
#include <conversions.h>

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

// MS5837 Depth Sensor
#define MS5837_SIG_IDX      (0x0008)        // Valid calibration signature
#define MS5837_ATM_LSB_IDX  (0x0009)        // Atmospheric pressure lower bytes
#define MS5837_ATM_MSB_IDX  (0x000A)        // Atmospheric pressure upper bytes
#define MS5837_FDN_LSB_IDX  (0x000B)        // Fluid density lower bytes
#define MS5837_FDN_MSB_IDX  (0x000C)        // Fluid density upper bytes
// -----------------------------------------------------------------------------

void calibration_load(void){
    calibration_load_bno055();
    calibration_load_ms5837();
}

void calibration_load_bno055(void){
    uint16_t sig;
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
    uint16_t sig;
    eeprom_read(MS5837_SIG_IDX, &sig);
    calibration_ms5837.valid = (sig == SIG_VALID);
    if(calibration_ms5837.valid){
        uint16_t atm_lsb, atm_msb, fdn_lsb, fdn_msb;
        eeprom_read(MS5837_ATM_LSB_IDX, &atm_lsb);
        eeprom_read(MS5837_ATM_MSB_IDX, &atm_msb);
        eeprom_read(MS5837_FDN_LSB_IDX, &fdn_lsb);
        eeprom_read(MS5837_FDN_MSB_IDX, &fdn_msb);
        uint8_t buf[4];
        buf[0] = atm_lsb & 0xFF;
        buf[1] = atm_lsb >> 8;
        buf[2] = atm_msb & 0xFF;
        buf[3] = atm_msb >> 8;
        calibration_ms5837.atm_pressure = conversions_data_to_float(buf, true);
        buf[0] = fdn_lsb & 0xFF;
        buf[1] = fdn_lsb >> 8;
        buf[2] = fdn_msb & 0xFF;
        buf[3] = fdn_msb >> 8;
        calibration_ms5837.fluid_density = conversions_data_to_float(buf, true);
    }
}

void calibration_store_ms5837(ms5837_cal_t new_data){
    uint8_t buf[4];

    conversions_float_to_data(new_data.atm_pressure, buf, true);
    uint16_t atm_lsb = (buf[1] << 8) | buf[0];
    uint16_t atm_msb = (buf[3] << 8) | buf[2];

    conversions_float_to_data(new_data.fluid_density, buf, true);
    uint16_t fdn_lsb = (buf[1] << 8) | buf[0];
    uint16_t fdn_msb = (buf[3] << 8) | buf[2];

    eeprom_write(MS5837_ATM_LSB_IDX, atm_lsb);
    eeprom_write(MS5837_ATM_MSB_IDX, atm_msb);
    eeprom_write(MS5837_FDN_LSB_IDX, fdn_lsb);
    eeprom_write(MS5837_FDN_MSB_IDX, fdn_msb);
    eeprom_write(MS5837_SIG_IDX, SIG_VALID);

    calibration_ms5837 = new_data;
    calibration_ms5837.valid = true;
}

void calibration_erase_ms5837(void){
    // Instead of actually erasing data (potentially more flash writes)
    // Just invalidate the signature (fewer flash writes / erases)
    eeprom_write(MS5837_SIG_IDX, 0x0001);
    calibration_ms5837.valid = false;
}
