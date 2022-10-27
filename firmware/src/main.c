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
#include <flags.h>
#include <timers.h>
#include <usb.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Declared in flags.h
volatile uint16_t flags_main = 0;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(void){
    clocks_init();
    ports_init();
    timers_init();
    dotstar_init();
    usb_init();

    bool toggle = false;
    dotstar_set(0, 0, 0);
    while(1){
        if(FLAG_CHECK(flags_main, FLAG_MAIN_10MS)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_10MS);
            // -----------------------------------------------------------------
            // Runs every 10ms
            // -----------------------------------------------------------------
            // Nothing here
            // -----------------------------------------------------------------
        }else if(FLAG_CHECK(flags_main, FLAG_MAIN_20MS)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_20MS);
            // -----------------------------------------------------------------
            // Runs every 20ms
            // -----------------------------------------------------------------
            // Nothing here
            // -----------------------------------------------------------------
        }else if(FLAG_CHECK(flags_main, FLAG_MAIN_50MS)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_50MS);
            // -----------------------------------------------------------------
            // Runs every 50ms
            // -----------------------------------------------------------------
            // Nothing here
            // -----------------------------------------------------------------
        }else if(FLAG_CHECK(flags_main, FLAG_MAIN_100MS)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_100MS);
            // -----------------------------------------------------------------
            // Runs every 100ms
            // -----------------------------------------------------------------
            // Nothing here
            // -----------------------------------------------------------------
        }else if(FLAG_CHECK(flags_main, FLAG_MAIN_1000MS)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_1000MS);
            // -----------------------------------------------------------------
            // Runs every 1000ms
            // -----------------------------------------------------------------
            // Blink dotstar LED
            if(toggle){
                dotstar_set(0, 0, 0);
            }else{
                dotstar_set(32, 0, 0);
            }
            toggle = !toggle;
            // -----------------------------------------------------------------
        }
        usb_process();
    }
}

