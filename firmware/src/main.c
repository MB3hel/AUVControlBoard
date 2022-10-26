/**
 * Program entry point and main tree
 * @file main.c
 * @author Marcus Behel
 */

#include <sam.h>
#include <clocks.h>
#include <ports.h>
#include <stdint.h>
#include <dotstar.h>


int main(void){
    clocks_init();
    ports_init();
    dotstar_init();

    uint8_t state = 0;
    while(1){
        switch(state){
        case 0:
            dotstar_set(100, 0, 0);
            break;
        case 1:
            dotstar_set(76, 20, 0);
            break;
        case 2:
            dotstar_set(50, 50, 0);
            break;
        case 3:
            dotstar_set(0, 100, 0);
            break;
        case 4:
            dotstar_set(0, 0, 100);
            break;
        case 5:
            dotstar_set(10, 0, 112);
            break;
        }
        state = (state + 1) % 6;
        delay_ms(250);
    }
}

