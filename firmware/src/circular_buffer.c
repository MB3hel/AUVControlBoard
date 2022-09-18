/**
 * @file circular_buffer.c
 * @author Marcus Behel (mgbehel@ncsu.edu)
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

#include <circular_buffer.h>


////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////

void cb_init(volatile circular_buffer *cb, volatile uint8_t *backing_array, unsigned int len){
    cb->array = backing_array;
    cb->size = len;
    cb->count = 0;
    cb->write_pos = 0;
    cb->read_pos = 0;
}

bool cb_write(volatile circular_buffer *cb, uint8_t src){
    if(CB_FULL(cb))
        return false;
    cb->array[cb->write_pos] = src;
    cb->count++;
    cb->write_pos++;
    if(cb->write_pos == cb->size)
        cb->write_pos = 0;
    return true;
}

bool cb_read(volatile circular_buffer *cb, uint8_t *dest){
    if(CB_EMPTY(cb))
        return false;
    *dest = cb->array[cb->read_pos];
    cb->count--;
    cb->read_pos++;
    if(cb->read_pos == cb->size)
        cb->read_pos = 0;
    return true;
}
