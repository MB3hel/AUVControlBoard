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
#include <motor_control.h>
#include <flags.h>
#include <i2c0.h>
#include <timers.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Definition of main flags field (see flags.h)
uint8_t flags_main = 0;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Program entry point
 */
int main(void){
    bool pccomm_initialized = false;
    uint8_t msg[PCCOMM_MAX_MSG_LEN];
    uint32_t msg_len;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Initialization
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    atmel_start_init();                             // Initialize ASF4 drivers & middleware
    dotstar_init();                                 // Initialize RGB led driver
    motor_pwm_init();                               // Initialize motor pwm configuration
    motor_control_init();                           // Initialize motor control
    conversions_init();                             // Initialize conversions helper
    cmdctrl_init();                                 // Initialize cmd & ctrl system
    i2c0_init();                                    // Initialize i2c0
    // TODO: Initialize I2C sensorss
    timers_init();                                  // Initialize timers (including WDT)


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Main loop
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    while (true) {
        if(FLAG_CHECK(flags_main, FLAG_MAIN_PCCOMM_MSG)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_PCCOMM_MSG);
            // ---------------------------------------------------------------------------------------------------------
            // Runs when a new message from the controlling pc is available
            // ---------------------------------------------------------------------------------------------------------
            msg_len = pccomm_get_msg(msg);
            cmdctrl_handle_msg(msg, msg_len);
            // ---------------------------------------------------------------------------------------------------------
        }else if(FLAG_CHECK(flags_main, FLAG_MAIN_PCCOMM_PROC)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_PCCOMM_PROC);
            // ---------------------------------------------------------------------------------------------------------
            // Runs when pccomm_process needs to be called by main
            // ---------------------------------------------------------------------------------------------------------
            pccomm_process();
            // ---------------------------------------------------------------------------------------------------------
        }else if(FLAG_CHECK(flags_main, FLAG_MAIN_10MS)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_10MS);
            // ---------------------------------------------------------------------------------------------------------
            // Runs every 10ms
            // ---------------------------------------------------------------------------------------------------------
            wdt_feed(&WDT_0);                           // Feed watchdog every 10ms
            // ---------------------------------------------------------------------------------------------------------
        }else if(FLAG_CHECK(flags_main, FLAG_MAIN_100MS)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_100MS);
            // ---------------------------------------------------------------------------------------------------------
            // Runs every 100ms
            // ---------------------------------------------------------------------------------------------------------
            
            // Attempt to initialize pccomm if not yet initialized
            if(!pccomm_initialized)
                pccomm_initialized = pccomm_init();

            // Update RGB LED to indicate cmdctrl mode
            switch(cmdctrl_get_mode()){
            case CMDCTRL_MODE_RAW:
                dotstar_set(100, 100, 0);
                break;
            case CMDCTRL_MODE_LOCAL:
                dotstar_set(10, 0, 100);
                break;
            }

            // Handle motor watchdog
            if(motor_control_watchdog_count()){
                cmdctrl_motors_killed();
            }
            // ---------------------------------------------------------------------------------------------------------
        }else if(FLAG_CHECK(flags_main, FLAG_MAIN_1000MS)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_1000MS);
            // ---------------------------------------------------------------------------------------------------------
            // Runs every 1000ms
            // ---------------------------------------------------------------------------------------------------------
            // Nothing here
            // ---------------------------------------------------------------------------------------------------------
        }else if(FLAG_CHECK(flags_main, FLAG_MAIN_I2C0_PROC)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_I2C0_PROC);
            // ---------------------------------------------------------------------------------------------------------
            // Runs when i2c0_process needs to be called by main
            // ---------------------------------------------------------------------------------------------------------
            i2c0_process();
            // ---------------------------------------------------------------------------------------------------------
        }else if(FLAG_CHECK(flags_main, FLAG_MAIN_I2C0_DONE)){
            // ---------------------------------------------------------------------------------------------------------
            // Runs when i2c0 completes a transaction
            // ---------------------------------------------------------------------------------------------------------
            // TODO: Have any sensor that uses i2c0 check if it's transaction is complete
            // ---------------------------------------------------------------------------------------------------------
        }else{
            // Enter sleep mode because nothing to do right now (no flags set)
            // Will be woken by ISRs, which may have set flags
            sleep(PM_SLEEPCFG_SLEEPMODE_IDLE0);
            // Will resume running here once woken by ISR, which may have set a flag
        }        
    }
}
