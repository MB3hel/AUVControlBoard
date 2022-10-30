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
#include <util.h>
#include <conversions.h>
#include <cmdctrl.h>
#include <motor_control.h>
#include <i2c0.h>
#include <bno055.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Declared in flags.h
volatile uint16_t flags_main = 0;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Blink LED on sensor error
 */
void sensor_error(void){
    bool toggle = false;
    while(1){
        if(FLAG_CHECK(flags_main, FLAG_MAIN_1000MS)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_1000MS);
            toggle = !toggle;
            if(toggle)
                dotstar_set(255, 32, 0);
            else
                dotstar_set(0, 0, 0);
        }else if (FLAG_CHECK(flags_main, FLAG_MAIN_10MS)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_10MS);
            TIMERS_WDT_FEED();
        }
        usb_process();
    }
}

int main(void){
    uint8_t msg[USB_MAX_MSG_LEN];
    uint32_t msg_len;
    
    // -----------------------------------------------------------------------------------------------------------------
    // Initialization
    // -----------------------------------------------------------------------------------------------------------------
    clocks_init();                                              // Initialize clocks
    ports_init();                                               // Initialize ports
    timers_init();                                              // Initialize timers
    dotstar_init();                                             // Initialize dotstar
    conversions_init();                                         // Initialize conversions
    motor_control_init();                                       // Initialize motor control
    cmdctrl_init();                                             // Initialize command & control
    usb_init();                                                 // Initialize USB
    i2c0_init();                                                // Initialize I2C
    delay_ms(100);                                              // Give sensors time to power on    
    if(!bno055_init()){                                         // Attempt BNO055 Init
        sensor_error();                                         // Error if no BNO055
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Main loop
    // -----------------------------------------------------------------------------------------------------------------
    while(1){
        // Handle flags
        if(FLAG_CHECK(flags_main, FLAG_MAIN_10MS)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_10MS);
            // ---------------------------------------------------------------------------------------------------------
            // Runs every 10ms
            // ---------------------------------------------------------------------------------------------------------
            TIMERS_WDT_FEED();
            // ---------------------------------------------------------------------------------------------------------
        }
        if(FLAG_CHECK(flags_main, FLAG_MAIN_20MS)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_20MS);
            // ---------------------------------------------------------------------------------------------------------
            // Runs every 20ms
            // ---------------------------------------------------------------------------------------------------------
            cmdctrl_update_motors();                    // Update motors periodically if needed
            // ---------------------------------------------------------------------------------------------------------
        }
        if(FLAG_CHECK(flags_main, FLAG_MAIN_50MS)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_50MS);
            // ---------------------------------------------------------------------------------------------------------
            // Runs every 50ms
            // ---------------------------------------------------------------------------------------------------------
            // Nothing here
            // ---------------------------------------------------------------------------------------------------------
        }
        if(FLAG_CHECK(flags_main, FLAG_MAIN_100MS)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_100MS);
            // ---------------------------------------------------------------------------------------------------------
            // Runs every 100ms
            // ---------------------------------------------------------------------------------------------------------
            cmdctrl_update_led();
            
            // Handle motor watchdog
            if(motor_control_watchdog_count()){
                cmdctrl_motors_killed();
            }
            // ---------------------------------------------------------------------------------------------------------
        }
        if(FLAG_CHECK(flags_main, FLAG_MAIN_1000MS)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_1000MS);
            // ---------------------------------------------------------------------------------------------------------
            // Runs every 1000ms
            // ---------------------------------------------------------------------------------------------------------
            // Nothing here
            // ---------------------------------------------------------------------------------------------------------
        }
        if(FLAG_CHECK(flags_main, FLAG_MAIN_USBMSG)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_USBMSG);
            // ---------------------------------------------------------------------------------------------------------
            // Runs when a message is in the usb message queue
            // ---------------------------------------------------------------------------------------------------------
            msg_len = usb_getmsg(msg);
            if(msg_len > 0)
                cmdctrl_handle_msg(msg, msg_len);
            // ---------------------------------------------------------------------------------------------------------
        }
        if(FLAG_CHECK(flags_main, FLAG_MAIN_I2C0_DONE)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_I2C0_DONE);
            // ---------------------------------------------------------------------------------------------------------
            // Runs when i2c0 finishes a transaction
            // ---------------------------------------------------------------------------------------------------------
            if(i2c0_curr_trans == &bno055_trans)
                bno055_i2c_done();
            // ---------------------------------------------------------------------------------------------------------
        }
        if(FLAG_CHECK(flags_main, FLAG_MAIN_BNO055_DELAY)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_BNO055_DELAY);
            // ---------------------------------------------------------------------------------------------------------
            // Runs when bno055 delay finishes
            // ---------------------------------------------------------------------------------------------------------
            bno055_delay_done();
            // ---------------------------------------------------------------------------------------------------------
        }
        if(FLAG_CHECK(flags_main, FLAG_MAIN_I2C0_TIMEOUT)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_I2C0_TIMEOUT);
            // ---------------------------------------------------------------------------------------------------------
            // Runs when i2c0 timeout occurs
            // ---------------------------------------------------------------------------------------------------------
            i2c0_timeout();
            // ---------------------------------------------------------------------------------------------------------
        }

        // Always process usb (allows tinyusb to handle events)
        usb_process();
    }
}

/**
 * Hard Fault Handler
 */
void HardFault_Handler(void){
    dotstar_set(255, 0, 0);                                     // LED red to indicate hard fault
    while(1);                                                   // Block forever. WDT should reset system.
}

/**
 * Memory Management Fault Handler
 */
void MemManagement_Handler(void){
    dotstar_set(255, 0, 0);                                     // LED red to indicate hard fault
    while(1);                                                   // Block forever. WDT should reset system.
}

/**
 * Bus Fault Handler
 */
void BusFault_Handler(void){
    dotstar_set(255, 0, 0);                                     // LED red to indicate hard fault
    while(1);                                                   // Block forever. WDT should reset system.
}

/**
 * Usage Fault Handler
 */
void UsageFault_Handler(void){
    dotstar_set(255, 0, 0);                                     // LED red to indicate hard fault
    while(1);                                                   // Block forever. WDT should reset system.
}
