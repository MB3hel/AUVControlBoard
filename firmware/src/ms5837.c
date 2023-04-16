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


#include <ms5837.h>
#include <i2c.h>
#include <stdint.h>
#include <simulator.h>

#define MS5837_ADDR                     0x76
#define MS5837_30BA_VER                 0x1A   // Sensor version ID for 30BA

#define MS5837_CMD_RESET                0x1E
#define MS5837_CMD_ADC_READ             0x00
#define MS5837_CMD_PROM_READ            0xA0
#define MS5837_CMD_CONVERT_D1_OSR256    0x40
#define MS5837_CMD_CONVERT_D1_OSR512    0x42
#define MS5837_CMD_CONVERT_D1_OSR1024   0x44
#define MS5837_CMD_CONVERT_D1_OSR2048   0x46
#define MS5837_CMD_CONVERT_D1_OSR4096   0x48
#define MS5837_CMD_CONVERT_D1_OSR8192   0x4A
#define MS5837_CMD_CONVERT_D2_OSR256    0x50
#define MS5837_CMD_CONVERT_D2_OSR512    0x52
#define MS5837_CMD_CONVERT_D2_OSR1024   0x54
#define MS5837_CMD_CONVERT_D2_OSR2048   0x56
#define MS5837_CMD_CONVERT_D2_OSR4096   0x58
#define MS5837_CMD_CONVERT_D2_OSR8192   0x5A

#define WRITE_BUF_SIZE          16
#define READ_BUF_SIZE           16

// TODO: Make this configurable via pc command
// freshwater = 997.0f
// saltwater = 1029.0f
static const float fluid_density = 997.0f;

// TODO: Make this configurable via pc command
static const float atm_pressure = 101325.0f;

static i2c_trans trans;
static uint8_t write_buf[WRITE_BUF_SIZE];
static uint8_t read_buf[READ_BUF_SIZE];

#define ms5837_perform(x)           i2c_perform_retries((x), 20, 5)

// Note: Size of 8 is important because of how crc code works
static uint16_t prom_data[8];

/**
 * crc4 function as defined in sensor datasheet (p10)
 */
static uint8_t crc4(uint16_t *data){
    uint32_t count;
    uint32_t remainder = 0;
    uint8_t bit;
    data[0] = (data[0] & 0x0FFF);
    data[7] = 0;
    for(count = 0; count < 16; ++count){
        if(count & 0x1)  remainder ^= (uint16_t)(data[count >> 1] & 0x00FF);
        else             remainder ^= (uint16_t)(data[count >> 1] >> 8);
        for(bit = 8; bit > 0; --bit){
            if(remainder & 0x8000)  remainder = (remainder << 1) ^ 0x3000;
            else                    remainder = remainder << 1;
        }
    }
    remainder = (remainder >> 12) & 0x000F;
    return remainder ^ 0x00;
}

void ms5837_init(void){
    trans.address = MS5837_ADDR;
    trans.write_buf = write_buf;
    trans.read_buf = read_buf;
}

bool ms5837_configure(void){
    // Reset command
    trans.write_buf[0] = MS5837_CMD_RESET;
    trans.write_count = 1;
    trans.read_count = 0;
    if(!ms5837_perform(&trans))
        return false;
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Read all PROM bytes
    for(unsigned int i = 0; i < 7; ++i){
        trans.write_buf[0] = MS5837_CMD_PROM_READ + (i * 2);
        trans.write_count = 1;
        trans.read_count = 2;
        if(!ms5837_perform(&trans))
            return false;
        prom_data[i] = (trans.read_buf[0] << 8) | trans.read_buf[1];
    }

    // crc4 calculation & validate data
    uint8_t crc_read = prom_data[0] >> 12;
    uint8_t crc_calc = crc4(prom_data);

    return crc_read == crc_calc;
}

bool ms5837_read(ms5837_data *data){
    uint32_t d1, d2;

    // Convert D1
    trans.write_buf[0] = MS5837_CMD_CONVERT_D1_OSR1024;
    trans.write_count = 1;
    trans.read_count = 0;
    if(!ms5837_perform(&trans))
        return false;
    vTaskDelay(pdMS_TO_TICKS(20));          // Ensure conversion completes before reading ADC

    // Read ADC (D1 data)
    trans.write_buf[0] = MS5837_CMD_ADC_READ;
    trans.write_count = 1;
    trans.read_count = 3;
    if(!ms5837_perform(&trans))
        return false;
    d1 = (trans.read_buf[0] << 16) | (trans.read_buf[1] << 8) | trans.read_buf[2];

    // Convert D2
    trans.write_buf[0] = MS5837_CMD_CONVERT_D2_OSR1024;
    trans.write_count = 1;
    trans.read_count = 0;
    if(!ms5837_perform(&trans))
        return false;
    vTaskDelay(pdMS_TO_TICKS(20));          // Ensure conversion completes before reading ADC

    // Read ADC
    trans.write_buf[0] = MS5837_CMD_ADC_READ;
    trans.write_count = 1;
    trans.read_count = 3;
    if(!ms5837_perform(&trans))
        return false;
    d2 = (trans.read_buf[0] << 16) | (trans.read_buf[1] << 8) | trans.read_buf[2];

    ////////////////////////////////////////////////////////////////////////////
    // Calculate values (as described on pages 11-12 of sensor datasheet)
    ////////////////////////////////////////////////////////////////////////////

    int32_t dT = 0;
    int64_t SENS = 0;
    int64_t OFF = 0;
    int32_t TEMP, TEMP100_DIV_100;
    int32_t P;
    int32_t SENSi = 0;
    int32_t OFFi = 0;
    int32_t Ti = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;

    // Calculate temperature
    dT = d2 - ((uint32_t)prom_data[5]) * 256l;
    TEMP = 2000l + ((int64_t)dT) * prom_data[6] / 8388608LL;
    
    // Calculate temperature compensated pressure
    SENS = ((int64_t)prom_data[1]) * 32768l + (((int64_t)prom_data[3]) * dT) / 256l;
    OFF = ((int64_t)prom_data[2]) * 65536l + (((int64_t)prom_data[4]) * dT) / 128l;
    
    // Skip this because it would just be recalculated in second order
    // P = (d1 * SENS / (2097152l) - OFF) / 8192l;
    

    // -------------------------------------------------------------------------
    // Second order compensation
    // -------------------------------------------------------------------------
    TEMP100_DIV_100 = TEMP / 100;
    if(TEMP100_DIV_100 < 20){
        // Low temperature
        Ti = (3 * ((int64_t)dT) * ((int64_t)dT)) / 8589934592ll;
        OFFi = (3 * (TEMP - 2000) * (TEMP - 2000)) / 2;
        SENSi = (5 * (TEMP - 2000) * (TEMP - 2000)) / 8;
        if(TEMP100_DIV_100 < -15){
            // Very low temperature
            OFFi = OFFi + 7 * (TEMP + 1500l) * (TEMP + 1500l);
            SENSi = SENSi + 4 * (TEMP + 1500l) * (TEMP + 1500l);
        }
    }else{
        // High temperature
        Ti = 2 * (dT * dT) / 137438953472ll;
        OFFi = ((TEMP - 2000) * (TEMP - 2000)) / 16;
        SENSi = 0;
    }

    OFF2 = OFF-OFFi;
    SENS2 = SENS-SENSi;

    TEMP = (TEMP-Ti);
    P = ((d1 * SENS2) / 2097152l - OFF2) / 8192l;

    // P in mbar * 10
    // TEMP in celsius * 100
    // 1mbar = 100Pa -> P * 10 = Pa
    if(sim_hijacked){
        // Use data from simulator not depth sensor
        data->pressure_mbar = -999;
        data->pressure_mbar = -999;
        data->depth_m = sim_depth;
    }else{
        data->pressure_mbar = P / 10.0f;
        data->temperature_c = TEMP / 100.0f;
        data->depth_m = (atm_pressure - (P * 10.0f)) / (fluid_density * 9.80665);   // Negative for below surface of water
    }

    ////////////////////////////////////////////////////////////////////////////

    return true;
}
