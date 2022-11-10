
#include <util.h>
#include <string.h>

char *itoa(int value, char *str, int base){
    if(value == 0){
        str[0] = '0';
        str[1] = '\0';
    }else{
        // Construct string in reverse
        bool neg = (value < 0 && base == 10);
        if(neg)
            value *= -1;
        int i = 0;
        while(value > 0){
            int r = value % base;
            value /= base;
            str[i++] = (r > 9) ? (r + 'A' - 10) : (r + '0');
        }
        if(neg)
            str[i++] = '-';
        str[i++] = '\0';

        // Reverse the string
        int a = 0, b = strlen(str) - 1;
        char tmp;
        while(a < b){
            tmp = str[a];
            str[a] = str[b];
            str[b] = tmp;
            ++a;
            --b;
        }
    }
    return str;
}

static unsigned int rand_seed = 5323;

void srand(unsigned int seed){
    rand_seed = seed;
}

unsigned int rand(){
    rand_seed = (8253729 * rand_seed + 2396403); 
    return rand_seed  % 32767;
}

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

uint16_t crc16_ccitt(uint8_t *data, uint32_t len){
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