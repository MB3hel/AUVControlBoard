#include <framework.h>
#include <FreeRTOS.h>
#include <task.h>
#include <led.h>
#include <delay.h>
#include <math.h>
#include <usb.h>
#include <thruster.h>
#include <app.h>
#include <cmdctrl.h>
#include <conversions.h>
#include <motor_control.h>
#include <i2c.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Program Entry point / startup
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(void){
    // -------------------------------------------------------------------------
    // System & Peripheral Initialization
    // -------------------------------------------------------------------------
    init_frameworks();
    conversions_init();
    delay_init();
    led_init();
    usb_init();
    thruster_init();
    mc_init();
    cmdctrl_init();
    i2c_init();
    // -------------------------------------------------------------------------
    
    app_init();

    // -------------------------------------------------------------------------
    // RTOS Startup
    // -------------------------------------------------------------------------
    vTaskStartScheduler();
    // -------------------------------------------------------------------------

    // Start scheduler should never return. This should never run, but is
    // included to make debugging easier in case it does
    taskDISABLE_INTERRUPTS();
    led_set(255, 0, 0);
    while(1){
        asm("nop");
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
