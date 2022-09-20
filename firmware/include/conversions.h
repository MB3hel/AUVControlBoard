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
 * Initialize conversions block
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
