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


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Definition of main flags field (see flags.h)
uint8_t flags_main = 0;


// Timer tasks
static struct timer_task task_10ms, task_100ms, task_1000ms; 


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Callback for timing flag tasks
 * This is called from timer ISR. Treat this function as an ISR.
 * System will wake from sleep after this function is called
 */
static void cb_timing(const struct timer_task *const timer_task){
    if(timer_task == &task_10ms){
        FLAG_SET(flags_main, FLAG_MAIN_10MS);
    }else if(timer_task == &task_100ms){
        FLAG_SET(flags_main, FLAG_MAIN_100MS);
    }else if(timer_task == &task_1000ms){
        FLAG_SET(flags_main, FLAG_MAIN_1000MS);
    }
}

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

    task_10ms.cb = cb_timing;                       // Setup 10ms timing task
    task_10ms.interval = 10;
    task_10ms.mode = TIMER_TASK_REPEAT;
    timer_add_task(&TIMER_0, &task_10ms);

    task_100ms.cb = cb_timing;                      // Setup 100ms timing task
    task_100ms.interval = 100;
    task_100ms.mode = TIMER_TASK_REPEAT;
    timer_add_task(&TIMER_0, &task_100ms);

    task_1000ms.cb = cb_timing;                     // Setup 1000ms timing task
    task_1000ms.interval = 1000;
    task_1000ms.mode = TIMER_TASK_REPEAT;
    timer_add_task(&TIMER_0, &task_1000ms);

    timer_start(&TIMER_0);                          // Start TIMER_0 (1ms timer)
    
    wdt_set_timeout_period(&WDT_0, 1024, 2000);     // Configure 2 second watchdog period
                                                    // Note: 1024Hz is CLK_WDT_OSC (not configurable on this chip)
                                                    // Only change the second parameter (2000)

	wdt_enable(&WDT_0);                             // Enable WDT


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
        }else{
            // Enter sleep mode because nothing to do right now (no flags set)
            // Will be woken by ISRs, which may have set flags
            sleep(PM_SLEEPCFG_SLEEPMODE_IDLE0);
            // Will resume running here once woken by ISR, which may have set a flag
        }        
    }
}
