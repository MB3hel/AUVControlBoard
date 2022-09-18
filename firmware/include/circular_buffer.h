/**
 * @file circular_buffer.h
 * @brief Ciruclar (ring) buffer implementation for comm protocols
 * Implemented to be ISR safe (uses volatile)
 * Read and write functions are inlinable
 * @author Marcus Behel (mgbehel@ncsu.edu)
 * @version 1.0.0
 */

/*
 * MIT License
 *
 * Copyright (c) 2022 Marcus Behel
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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
