/**
 * Program entry point, main tree, and ISRs
 * @file main.c
 * @author Marcus Behel
 */

#include <atmel_start.h>
#include <pccomm.h>
#include <motor_pwm.h>
#include <conversions.h>
#include <cmdctrl.h>
#include <stdbool.h>
#include <dotstar.h>


/**
 * Program entry point
 */
int main(void){
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Initialization
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    atmel_start_init();                             // Initialize ASF4 drivers & middleware
    dotstar_init();                                 // Initialize RGB led driver
    pccomm_init();                                  // Initialize USB communications with PC
    motor_pwm_init();                               // Initialize motor pwm configuration
    conversions_init();                             // Initialize conversions helper
    cmdctrl_init();                                 // Initialize cmd & ctrl system

    timer_start(&TIMER_0);                          // Start TIMER_0 (1ms timer)
    
    wdt_set_timeout_period(&WDT_0, 1000, 5000);     // Configure 5 second watchdog period
	wdt_enable(&WDT_0);                             // Enable WDT

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Main loop
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    while (true) {
        wdt_feed(&WDT_0);
    }
}
