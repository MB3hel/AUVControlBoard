/**
 * Timer configuration
 * 
 * @file timers.h
 * @author Marcus Behel
 */

#pragma once

#include <stdint.h>


// Feed by writing 0xA5
// Reset by writing anything else
// See p247 in datasheet
#define TIMERS_WDT_FEED()                           WDT->CLEAR.reg = 0xA5; while(WDT->SYNCBUSY.bit.CLEAR);
#define TIMERS_WDT_RESET_NOW()                      WDT->CLEAR.reg = 0xDC; while(WDT->SYNCBUSY.bit.CLEAR);


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Initialize all timers
 */
void timers_init(void);

/**
 * Set thruster speeds using PWM signals
 * @param speeds Array of 8 speeds (-1.0 to 1.0)
 */
void timers_thruster_pwm_set(float *speeds);

/**
 * Start BNO055 delay
 * @param delay Duration in ms to delay for
 */
void timers_bno055_delay(uint32_t delay);

/**
 * Set timeout for I2C0
 * @param us Number of us to wait for. Set to 0 to disable.
 */
void timers_i2c0_timeout(uint32_t us);

