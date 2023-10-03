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

#include <util/conversions.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static bool big_endian;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void conversions_init(void){
    // i = 0x0001
    //  On big endian systems this is stored 0x00, 0x01
    // c = pointer to leftmost byte (array)
    // c[0] = leftmost byte. 1 on little endian. 0 on big endian
    uint16_t i = 1;
    uint8_t *c = (uint8_t*)&i;
    big_endian = !c[0];
}

void conversions_int32_to_data(int32_t input, uint8_t *outBuffer, bool littleEndian){
    if(littleEndian){
        outBuffer[0] = input;
        outBuffer[1] = input >> 8;
        outBuffer[2] = input >> 16;
        outBuffer[3] = input >> 24;
    }else{
        outBuffer[0] = input >> 24;
        outBuffer[1] = input >> 16;
        outBuffer[2] = input >> 8;
        outBuffer[3] = input;
    }
}

int32_t conversions_data_to_int32(uint8_t *data, bool littleEndian){
    if(littleEndian){
        return (int32_t)data[0] | 
               (int32_t)data[1] << 8 | 
               (int32_t)data[2] << 16 |
               (int32_t)data[3] << 24;
    }else{
        return (int32_t)data[0] << 24| 
               (int32_t)data[1] << 16 | 
               (int32_t)data[2] << 8 |
               (int32_t)data[3];
    }
}

void conversions_int16_to_data(int16_t input, uint8_t *outBuffer, bool littleEndian){
    if(littleEndian){
        outBuffer[0] = input;
        outBuffer[1] = input >> 8;
    }else{
        outBuffer[0] = input >> 8;
        outBuffer[1] = input;
    }
}

int16_t conversions_data_to_int16(uint8_t *data, bool littleEndian){
    if(littleEndian){
        return data[0] | data[1] << 8;
    }else{
        return data[0] << 8 | data[1];
    }
}

void conversions_float_to_data(float input, uint8_t *outBuffer, bool littleEndian){
    // Pointer to leftmost byte of input
    uint8_t *ptr = (uint8_t*)&input;
    
    if(big_endian == littleEndian){
        // System endianess and desired endianess different, so reverse order
        outBuffer[0] = ptr[3];
        outBuffer[1] = ptr[2];
        outBuffer[2] = ptr[1];
        outBuffer[3] = ptr[0];
    }else{
        // System endianess and desired endianess match, so no need to reverse order
        outBuffer[0] = ptr[0];
        outBuffer[1] = ptr[1];
        outBuffer[2] = ptr[2];
        outBuffer[3] = ptr[3];
    }
}

float conversions_data_to_float(uint8_t *data, bool littleEndian){
    uint8_t dataRaw[4];
    
    if(big_endian == littleEndian){
        // System endianess and desired endianess different, so reverse order
        dataRaw[0] = data[3];
        dataRaw[1] = data[2];
        dataRaw[2] = data[1];
        dataRaw[3] = data[0];
    }else{
        // System endianess and desired endianess match, so no need to reverse order
        dataRaw[0] = data[0];
        dataRaw[1] = data[1];
        dataRaw[2] = data[2];
        dataRaw[3] = data[3];
    }

    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wstrict-aliasing"

    // Dereference pointer to start of dataRaw interpreted as float
    return *((float*)(dataRaw));

    #pragma GCC diagnostic pop
}
