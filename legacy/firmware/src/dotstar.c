/**
 * @file dotstar.c
 * @author Marcus Behel
 */

#include <dotstar.h>
#include <ports.h>
#include <clocks.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void write_byte(uint8_t val){
    for (uint8_t bit = 8; bit > 0; bit--){
        ports_gpio_write(P_DS_DAT, (val & (1 << (bit - 1))) ? true : false);
        ports_gpio_set(P_DS_CLK);
        delay_us(1);
        ports_gpio_clear(P_DS_CLK);
        delay_us(1);
    }
}

void dotstar_init(void){
    // Pin setup taken care of by ports_init
    // Nothing needed here
}

void dotstar_set(uint8_t r, uint8_t g, uint8_t b){
    write_byte(0x00);
    write_byte(0x00);
    write_byte(0x00);
    write_byte(0x00);
    write_byte(0xFF);
    write_byte(b);
    write_byte(g);
    write_byte(r);
    write_byte(0xFF);
}
