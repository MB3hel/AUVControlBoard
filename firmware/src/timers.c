/**
 * @file timers.c
 * @author Marcus Behel
 */

#include <timers.h>
#include <atmel_start.h>
#include <flags.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Timer tasks
static struct timer_task task_10ms, task_100ms, task_1000ms;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void cb_timing(const struct timer_task *const timer_task){
    if(timer_task == &task_10ms){
        FLAG_SET(flags_main, FLAG_MAIN_10MS);
    }else if(timer_task == &task_100ms){
        FLAG_SET(flags_main, FLAG_MAIN_100MS);
    }else if(timer_task == &task_1000ms){
        FLAG_SET(flags_main, FLAG_MAIN_1000MS);
    }
}

void timers_init(void){
    // -----------------------------------------------------------------------------------------------------------------
    // TIMER_0 (1ms timer)
    // -----------------------------------------------------------------------------------------------------------------
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
    // -----------------------------------------------------------------------------------------------------------------

    // -----------------------------------------------------------------------------------------------------------------
    // WDT
    // -----------------------------------------------------------------------------------------------------------------
    wdt_set_timeout_period(&WDT_0, 1024, 2000);     // Configure 2 second watchdog period
                                                    // Note: 1024Hz is CLK_WDT_OSC (not configurable on this chip)
                                                    // Only change the second parameter (2000)

	wdt_enable(&WDT_0);                             // Enable WDT
    // -----------------------------------------------------------------------------------------------------------------
}
