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
