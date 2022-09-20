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


int main(void){
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Initialization
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    atmel_start_init();                             // Initialize ASF4 drivers & middleware
    pccomm_init();                                  // Initialize USB communications with PC
    motor_pwm_init();                               // Initialize motor pwm configuration
    conversions_init();                             // Initilize conversions helper
    cmdctrl_init();                                 // Initilize cmd & ctrl system
    

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Main loop
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    while (1) {
        
    }
}
