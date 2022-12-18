#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <FreeRTOS.h>
#include <task.h>


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

/**
 * Perform an i2c transactions with multiple attempts if it fails
 * 
 * @param trans Pointer to transaction to perform
 * @param delay_ms Time in milliseconds between retries
 * @param max_retires Max number of attempts
 * @return true on success; false if all retries fail
 */
bool i2c_perform_retries(i2c_trans *trans, unsigned int delay_ms, unsigned int max_retires);
