#pragma once

#include <stdint.h>
#include <stdbool.h>


typedef struct {
    // Address of device for transaction
    uint8_t address;
    
    // Buffer holding data to be written (set before perform)
    // Count is number of bytes to write (set before perform)
    uint8_t *write_buf;
    unsigned int write_count;

    // Buffer holding read data
    // Count is number of bytes to read (set before perform)
    uint8_t *read_buf;
    unsigned int read_count;
} i2c_trans;

/**
 * Initialize I2C bus in master mode
 */
void i2c_init(void);

/**
 * Perform an i2c transaction
 * @param trans Pointer to transaction to perform
 * @return true on success, false on failure
 */
bool i2c_perform(i2c_trans *trans);
