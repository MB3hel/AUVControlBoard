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

/**
 * @file conversions.h
 * 
 * Data conversions helpers
 * 
 * @author Marcus Behel
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Initialize conversions helpers
 */
void conversions_init(void);

/**
 * Convert 32-bit int to bytes
 * @param input Integer to convert
 * @param outBuffer Where to output data
 * @param littleEndian Should convert little endian
 */
void conversions_int32_to_data(int32_t input, uint8_t *outBuffer, bool littleEndian);

/**
 * Convert data to 32-bit int
 * @param data Bytes to convert
 * @param littleEndian Should convert little endian
 * @return Converted integer
 */
int32_t conversions_data_to_int32(uint8_t *data, bool littleEndian);

/**
 * Convert 16-bit int to bytes
 * @param input Integer to convert
 * @param outBuffer Where to output data
 * @param littleEndian Should convert little endian
 */
void conversions_int16_to_data(int16_t input, uint8_t *outBuffer, bool littleEndian);

/**
 * Convert data to 16-bit int
 * @param data Bytes to convert
 * @param littleEndian Should convert little endian
 * @return Converted integer
 */
int16_t conversions_data_to_int16(uint8_t *data, bool littleEndian);

/**
 * Convert float to bytes
 * @param input Integer to convert
 * @param outBuffer Where to output data
 * @param littleEndian Should convert little endian
 */
void conversions_float_to_data(float input, uint8_t *outBuffer, bool littleEndian);

/**
 * Convert data to float
 * @param data Bytes to convert
 * @param littleEndian Should convert little endian
 * @return Converted float
 */
float conversions_data_to_float(uint8_t *data, bool littleEndian);
