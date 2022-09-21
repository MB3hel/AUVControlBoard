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


/**
 * Program entry point
 */
int main(void){
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Initialization
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    atmel_start_init();                             // Initialize ASF4 drivers & middleware
    pccomm_init();                                  // Initialize USB communications with PC
    motor_pwm_init();                               // Initialize motor pwm configuration
    conversions_init();                             // Initialize conversions helper
    cmdctrl_init();                                 // Initialize cmd & ctrl system

    timer_start(&TIMER_0);                          // Start TIMER_0 (1ms timer)
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Main loop
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    while (true) {
        
    }
}
