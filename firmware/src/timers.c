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
static struct timer_task task_10ms, task_20ms, task_50ms, task_100ms, task_1000ms, task_bno055, task_safe_delay;

static volatile bool safe_delay_done;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void cb_timing(const struct timer_task *const timer_task){
    if(timer_task == &task_10ms){
        FLAG_SET(flags_main, FLAG_MAIN_10MS);
    }else if(timer_task == &task_20ms){
        FLAG_SET(flags_main, FLAG_MAIN_20MS);
    }else if(timer_task == &task_50ms){
        FLAG_SET(flags_main, FLAG_MAIN_50MS);
    }else if(timer_task == &task_100ms){
        FLAG_SET(flags_main, FLAG_MAIN_100MS);
    }else if(timer_task == &task_1000ms){
        FLAG_SET(flags_main, FLAG_MAIN_1000MS);
    }else if(timer_task == &task_bno055){
        FLAG_SET(flags_main, FLAG_MAIN_BNO055_DELAY);
        timer_remove_task(&TIMER_0, &task_bno055);
    }else if(timer_task == &task_safe_delay){
        safe_delay_done = true;
        timer_remove_task(&TIMER_0, &task_safe_delay);
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

    task_20ms.cb = cb_timing;                       // Setup 20ms timing task
    task_20ms.interval = 20;
    task_20ms.mode = TIMER_TASK_REPEAT;
    timer_add_task(&TIMER_0, &task_20ms);

    task_50ms.cb = cb_timing;                       // Setup 50ms timing task
    task_50ms.interval = 50;
    task_50ms.mode = TIMER_TASK_REPEAT;
    timer_add_task(&TIMER_0, &task_50ms);

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
}

void timers_wdt_enable(void){
    wdt_set_timeout_period(&WDT_0, 1024, 2000);     // Configure 2 second watchdog period
                                                    // Note: 1024Hz is CLK_WDT_OSC (not configurable on this chip)
                                                    // Only change the second parameter (2000)
	wdt_enable(&WDT_0);                             // Enable WDT
}

void timers_wdt_feed(void){
    wdt_feed(&WDT_0);
}

void timers_enable_bno055_delay(uint32_t delay){
    task_bno055.cb = cb_timing;
    task_bno055.interval = delay;
    task_bno055.mode = TIMER_TASK_ONE_SHOT;
    timer_add_task(&TIMER_0, &task_bno055);
}

void timers_safe_delay(uint32_t delayms){
    safe_delay_done = false;
    task_safe_delay.cb = cb_timing;
    task_safe_delay.interval = delayms;
    task_safe_delay.mode = TIMER_TASK_ONE_SHOT;
    timer_add_task(&TIMER_0, &task_safe_delay);
    while(!safe_delay_done){
        if(FLAG_CHECK(flags_main, FLAG_MAIN_10MS)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_10MS);
            timers_wdt_feed();
        }
    }
}
