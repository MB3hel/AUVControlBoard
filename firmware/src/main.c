/**
 * Program entry point and main tree
 * @file main.c
 * @author Marcus Behel
 */

#include <sam.h>
#include <clocks.h>
#include <ports.h>
#include <stdint.h>


int main(void){
    ports_pinfunc(P_RED_LED, PORT_PINFUNC_GPIO);
    ports_gpio_dir(P_RED_LED, PORT_GPIO_OUT);
    ports_gpio_clear(P_RED_LED);
    clocks_init();
    ports_gpio_set(P_RED_LED);
    while(1){
        ports_gpio_toggle(P_RED_LED);
        delay_cycles(CLOCKS_SEC_TO_CYCLES(1));
    }
}

