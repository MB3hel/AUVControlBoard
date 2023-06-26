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


bool calibration_valid = false;
// TODO: Other values


// Indices (addresses) of different values (16-bit) in eeprom
#define SIG_IDX     (0x00)      // First 2 values are signature
// TODO: Other values


void calibration_load(void){
    // First 2 half-words are a calibration signature. If the bytes match this signature,
    // it is assumed that a valid set of calibration constants are stored in the eeprom.
    // Otherwise, it is assumed that the calibration data is not valid (has never been stored).

    // This is the correct signature. Will be compared to read signature bytes
    const uint16_t cal_sig[4] = {0x3b3b, 0x3b3b};

    // Read signature from eeprom and compare
    uint16_t val;
    for(unsigned int i = 0; i < 2; ++i){
        eeprom_read(SIG_IDX + i, &val);
        if(val != cal_sig[i]){
            calibration_valid = false;
            return;
        }
    }
    calibration_valid = true;


    // TODO: Load other calibration values
}
