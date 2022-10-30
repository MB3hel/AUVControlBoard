#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifndef NULL
#define NULL    0
#endif

// I2C transaction status codes
#define I2C_STATUS_SUCCESS          0           // Transaction finished successfully
#define I2C_STATUS_BUSY             1           // Transaction in progress
#define I2C_STATUS_ERROR            2           // Transaction finished unsuccessfully
#define I2C_STATUS_TIMEOUT          3           // Transaction timed out (error)


typedef struct {
    uint8_t address;
    uint8_t *write_buf;
    uint32_t write_count;
    volatile uint8_t *read_buf;
    uint32_t read_count;
    volatile uint8_t status;
} i2c_trans;

/**
 * Seed pseudorandom number generator
 */
void srand(unsigned int seed);

/**
 * Generate a pseudorandom number (max of 32767)
 */
unsigned int rand();

/**
 * Check if two byte arrays are identical
 * @param a First byte array
 * @param len_a Length of first array
 * @param b Second byte array
 * @param len_b Length of second array
 * @return true If arrays match
 * @return false If arrays do not match
 */
bool data_matches(const uint8_t *a, uint32_t len_a, const uint8_t *b, uint32_t len_b);

/**
 * Check if one array starts with the data in another array
 * @param a The array to search in ("full" data)
 * @param len_a Length of array a
 * @param b The array to search for ("sub" / "prefix" data)
 * @param len_b Length of array b
 * @return true If array a starts with array b
 * @return false If array a does not start with array b
 */
bool data_startswith(const uint8_t *a, uint32_t len_a, const uint8_t *b, uint32_t len_b);

/**
 * Calculate 16-bit CRC (CCITT-FALSE) of the given data
 * @param data Data to calculate crc of
 * @param len Length of data
 * @return uint16_t Calculated crc
 */
uint16_t crc16_ccitt(uint8_t *data, uint32_t len);