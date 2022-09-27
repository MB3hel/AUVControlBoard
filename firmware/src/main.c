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


// General flags

#define FLAG_10MS       0b00000001
#define FLAG_100MS      0b00000010
#define FLAG_1000MS     0b00000100

uint8_t flags = 0;

#define SET_FLAG(x)         (flags |= x)
#define CLEAR_FLAG(x)       (flags &= ~x)
#define CHECK_FLAG(x)       (flags & x)


// Timer tasks

static struct timer_task task_10ms, task_100ms, task_1000ms; 


/**
 * Callback for timing flag tasks
 */
static void cb_timing(const struct timer_task *const timer_task){
    if(timer_task == &task_10ms){
        SET_FLAG(FLAG_10MS);
    }else if(timer_task == &task_100ms){
        SET_FLAG(FLAG_100MS);
    }else if(timer_task == &task_1000ms){
        SET_FLAG(FLAG_1000MS);
    }
}

/**
 * Program entry point
 */
int main(void){
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Initialization
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    bool pccomm_initialized = false;
    uint8_t msg[PCCOMM_MAX_MSG_LEN];
    uint32_t msg_len;

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
    task_100ms.interval = 10;
    task_100ms.mode = TIMER_TASK_REPEAT;
    timer_add_task(&TIMER_0, &task_100ms);

    task_1000ms.cb = cb_timing;                     // Setup 1000ms timing task
    task_1000ms.interval = 10;
    task_1000ms.mode = TIMER_TASK_REPEAT;
    timer_add_task(&TIMER_0, &task_1000ms);

    timer_start(&TIMER_0);                          // Start TIMER_0 (1ms timer)
    
    wdt_set_timeout_period(&WDT_0, 1000, 2000);     // Configure 2 second watchdog period
	wdt_enable(&WDT_0);                             // Enable WDT


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Main loop
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    while (true) {
        msg_len = pccomm_get_msg(msg);
        if(msg_len != 0){
            // ---------------------------------------------------------------------------------------------------------
            // Runs when a new message from the controlling pc is available
            // ---------------------------------------------------------------------------------------------------------
            cmdctrl_handle_msg(msg, msg_len);
            // ---------------------------------------------------------------------------------------------------------
        }else if(CHECK_FLAG(FLAG_10MS)){
            CLEAR_FLAG(FLAG_10MS);
            // ---------------------------------------------------------------------------------------------------------
            // Runs every 10ms
            // ---------------------------------------------------------------------------------------------------------
            // Nothing here
            // ---------------------------------------------------------------------------------------------------------
        }else if(CHECK_FLAG(FLAG_100MS)){
            CLEAR_FLAG(FLAG_100MS);
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
        }else if(CHECK_FLAG(FLAG_1000MS)){
            CLEAR_FLAG(FLAG_1000MS);
            // ---------------------------------------------------------------------------------------------------------
            // Runs every 1000ms
            // ---------------------------------------------------------------------------------------------------------
            // Nothing here
            // ---------------------------------------------------------------------------------------------------------
        }

        // Feed watchdog every iteration
        wdt_feed(&WDT_0);
    }
}
