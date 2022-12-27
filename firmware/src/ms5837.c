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

static i2c_trans trans;
static uint8_t write_buf[WRITE_BUF_SIZE];
static uint8_t read_buf[READ_BUF_SIZE];

#define ms57837_perform(x)           i2c_perform_retries((x), 20, 5)

void ms5837_init(void){
    trans.address = MS5837_ADDR;
    trans.write_buf = write_buf;
    trans.read_buf = read_buf;
}

bool ms5837_configure(void){

}

bool ms5837_read(ms5837_data *data){

}
