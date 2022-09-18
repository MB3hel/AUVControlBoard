/**
 * @file circular_buffer.c
 * @author Marcus Behel (mgbehel@ncsu.edu)
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

bool cb_read(volatile circular_buffer *cb, volatile uint8_t *dest){
    if(CB_EMPTY(cb))
        return false;
    *dest = cb->array[cb->read_pos];
    cb->count--;
    cb->read_pos++;
    if(cb->read_pos == cb->size)
        cb->read_pos = 0;
    return true;
}
