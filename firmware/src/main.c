/**
 * Program entry point and main tree
 * @file main.c
 * @author Marcus Behel
 */

#include <clocks.h>
#include <ports.h>

int main(void){
    ports_pinfunc(P_RED_LED, PORT_PINFUNC_GPIO);
    ports_gpio_dir(P_RED_LED, PORT_GPIO_OUT);
    ports_gpio_clear(P_RED_LED);
    clocks_init();
    while(1){

    }
}

