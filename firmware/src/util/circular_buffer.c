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

#include <util/circular_buffer.h>


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