/**
 * @file util.c
 * @author Marcus Behel
 */

#include <util.h>


bool data_matches(const uint8_t *a, uint32_t len_a, const uint8_t *b, uint32_t len_b){
    if(len_a != len_b)
        return false;
    for(uint32_t i = 0; i < len_a; ++i){
        if(a[i] != b[i])
            return false;
    }
    return true;
}

bool data_startswith(const uint8_t *a, uint32_t len_a, const uint8_t *b, uint32_t len_b){
    if(len_a < len_b)
        return false;
    for(uint32_t i = 0; i < len_b; ++i){
        if(a[i] != b[i])
            return false;
    }
    return true;
}

uint16_t crc16_ccitt(volatile uint8_t *data, uint32_t len){
    uint16_t crc = 0xFFFF;
    int pos = 0;
    while(pos < len){
        uint8_t b = data[pos];
        for(int i = 0; i < 8; ++i){
            uint8_t bit = ((b >> (7 - i) & 1) == 1);
            uint8_t c15 = ((crc >> 15 & 1) == 1);
            crc <<= 1;
            if(c15 ^ bit){
                crc ^= 0x1021;
            }
        }
        pos++;
    }
    return crc;
}

void vmemcpy(volatile void *dest, volatile void *src, size_t len){
    volatile uint8_t *bdest = (uint8_t*)dest;
    volatile uint8_t *bsrc = (uint8_t*)src;
    for(size_t i = 0; i < len; ++i){
        bdest[i] = bsrc[i];
    }
}
