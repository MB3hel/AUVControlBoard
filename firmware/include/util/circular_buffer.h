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

#pragma once

#include <stdint.h>
#include <stdbool.h>

////////////////////////////////////////////////////////////////////////////////
/// Typedefs
////////////////////////////////////////////////////////////////////////////////

typedef struct {
    volatile uint8_t *array;    // Array backing the buffer
    unsigned int size;          // Size of backing array
    unsigned int count;         // Number of bytes in buffer
    unsigned int write_pos;     // Position to write data at
    unsigned int read_pos;      // Position to read data from
} circular_buffer;


////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////

// Check if buffer is full
#define CB_FULL(cb)         ((cb)->count == (cb)->size)

// Check if buffer is empty
#define CB_EMPTY(cb)        ((cb)->count == 0)

// Number of bytes that can be written until buffer is full
#define CB_AVAIL_WRITE(cb)  ((cb)->size - (cb)->count)

// Number of bytes that can be read until buffer is empty
#define CB_AVAIL_READ(cb)   ((cb)->count)


////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////

/**
 * Initialize a circular buffer with the given backing array
 * @param cb Circular buffer to initialize
 * @param backing_array Array to back the buffer
 * @param len Length of array backing the buffer
 */
void cb_init(volatile circular_buffer *cb, volatile uint8_t *backing_array, unsigned int len);

/**
 * Write a byte into the buffer
 * @param cb Buffer to write into
 * @param src Byte to write
 * @return true If successfully written
 * @return false If buffer is full
 */
bool cb_write(volatile circular_buffer *cb, uint8_t src);

/**
 * Read a byte from the bufffer
 * @param cb Circular buffer to read from
 * @param dest Pointer to destination to move read byte into
 * @return true If successfully read
 * @return false If buffer is empty
 */
bool cb_read(volatile circular_buffer *cb, uint8_t *dest);
