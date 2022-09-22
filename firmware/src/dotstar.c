/**
 * @file dootstar.c
 * @author Marcus Behel
 */

#include <dotstar.h>
#include <atmel_start.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void dotstar_write_byte(uint8_t val){
    for (uint8_t bit = 0u; bit < 8u; bit++){
        gpio_set_pin_level(DS_DAT, (val & (1 << bit)) ? true : false);
        delay_us(1);
        gpio_set_pin_level(DS_CLK, true);
        delay_us(1);
        gpio_set_pin_level(DS_CLK, false);
        delay_us(1);
    }
}

void dotstar_init(void){
    dotstar_set(0, 0, 0);
    gpio_set_pin_level(DS_DAT, false);
    gpio_set_pin_level(DS_CLK, true);
}

void dotstar_set(uint8_t r, uint8_t g, uint8_t b){
    dotstar_write_byte(0xFF);
    dotstar_write_byte(r);
    dotstar_write_byte(g);
    dotstar_write_byte(b);
    dotstar_write_byte(0xFF);
}
