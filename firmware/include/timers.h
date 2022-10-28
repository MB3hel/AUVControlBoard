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
#define TIMERS_WDT_FEED()                           (WDT->CLEAR.reg = 0xA5)
#define TIMERS_WDT_RESET_NOW()                      (WDT->CLEAR.reg = 0xFF)


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
 * Setup i2c0 timeout
 * @param target Function to run on timeout
 * @param timeout Timeout in us
 */
void timers_i2c0_timeout_init(void (*target)(void), uint32_t timeout);

/**
 * Reset i2c0 timeout task to original timeout (enables timeout too)
 */
void timers_i2c0_timeout_reset(void);

/**
 * Disable i2c0 timeout
 */
void timers_i2c0_timeout_disable(void);
