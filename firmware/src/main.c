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
#include <ms5837.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// TODO: REMOVE THIS! DEBUG ONLY!
char tmpmsg[32];

// Declared in flags.h
volatile uint16_t flags_main = 0;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Starts i2c0 transactions as needed. Handles multiple sensors in such a way
 * that no sensor will be resource starved.
 * This is achieved by checking sensors in a fixed order, but starting at different points
 * Checking always starts after the sensor that just finished a transaction (not from the start of the list)
 * This ensures that any sensor will at most have to wait for every other sensor to complete one transaction
 * before it gets the bus.
 */
void i2c0_manager(void){
    #define BNO055      0
    #define MS5837      1
    #define NDEVICES    2

    static uint8_t idx = 0;

    if(FLAG_CHECK(flags_main, FLAG_MAIN_I2C0_TIMEOUT)){
        FLAG_CLEAR(flags_main, FLAG_MAIN_I2C0_TIMEOUT);
        // ---------------------------------------------------------------------------------------------------------
        // Runs when i2c0 timeout occurs
        // ---------------------------------------------------------------------------------------------------------
        i2c0_timeout();
        // ---------------------------------------------------------------------------------------------------------
    }

    if(FLAG_CHECK(flags_main, FLAG_MAIN_I2C0_DONE)){
        FLAG_CLEAR(flags_main, FLAG_MAIN_I2C0_DONE);
        // ---------------------------------------------------------------------------------------------------------
        // Runs when i2c0 finishes a transaction
        // ---------------------------------------------------------------------------------------------------------        
        if(i2c0_curr_trans == &bno055_trans){
            bno055_i2c_done();
            idx = BNO055 + 1;
        }else if(i2c0_curr_trans == &ms5837_trans){
            ms5837_i2c_done();
            idx = MS5837 + 1;
        }
        if(idx >= NDEVICES)
            idx = 0;
        // ---------------------------------------------------------------------------------------------------------
    }

    // Note I2C0 uses interrupts, thus it is possible done is set here and it becomes idle
    // between the above block of code and the below block of code

    // Start a new transaction if i2c0 is idle ONLY IF THE DONE FLAG IS NOT SET
    // CANNOT start a new transaction if the done flag is set because this means
    // that the sensor has not been notified that it's transaction finished
    // This will cause that sensor to stop working because the state machine will never
    // get the i2c_done / i2c_error trigger
    if(I2C0_IDLE && !FLAG_CHECK(flags_main, FLAG_MAIN_I2C0_DONE)){
        uint8_t tmp = idx;
        bool exit = false;
        do{
            switch(tmp){
            case BNO055:
                if(FLAG_CHECK(flags_main, FLAG_MAIN_BNO055_WANTI2C)){
                    // If starting transaction fails, leave flag set so it will try again
                    if(i2c0_start(&bno055_trans)){
                        FLAG_CLEAR(flags_main, FLAG_MAIN_BNO055_WANTI2C);
                    }
                    exit = true;
                }
                break;
            case MS5837:
                if(FLAG_CHECK(flags_main, FLAG_MAIN_MS5837_WANTI2C)){
                    // If starting transaction fails, leave flag set so it will try again
                    if(i2c0_start(&ms5837_trans)){
                        FLAG_CLEAR(flags_main, FLAG_MAIN_MS5837_WANTI2C);
                    }
                    exit = true;
                }
                break;
            }
            tmp++;
            if(tmp >= NDEVICES)
                tmp = 0;
        }while(tmp != idx && !exit);
    }
}

int main(void){

    uint8_t msg[USB_MAX_MSG_LEN];                               // Holds message received from usb
    uint32_t msg_len;                                           // Length of message received from usb

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
    bno055_init();                                              // Initialize BNO055
    ms5837_init();                                              // Initialize MS5837

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
            // cmdctrl_update_led();
            
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
        if(FLAG_CHECK(flags_main, FLAG_MAIN_BNO055_DELAY)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_BNO055_DELAY);
            // ---------------------------------------------------------------------------------------------------------
            // Runs when bno055 delay finishes
            // ---------------------------------------------------------------------------------------------------------
            bno055_delay_done();
            // ---------------------------------------------------------------------------------------------------------
        }
        if(FLAG_CHECK(flags_main, FLAG_MAIN_MS5837_DELAY)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_MS5837_DELAY);
            // ---------------------------------------------------------------------------------------------------------
            // Runs when ms5837 delay finishes
            // ---------------------------------------------------------------------------------------------------------
            dotstar_set(0, 0, 0);
            ms5837_delay_done();
            // ---------------------------------------------------------------------------------------------------------
        }

        // Always run i2c0 resource manager
        // The resource manager starts transactions for multiple sensors
        // in such a way that no sensor will be resource starved
        i2c0_manager();

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
