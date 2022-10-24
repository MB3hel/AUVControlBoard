/**
 * @file timers.h
 * Initialization and callbacks for timer tasks
 * @author Marcus Behel
 */

#pragma once

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Initialize timers subsystem
 */
void timers_init(void);

/**
 * Enable watchdog timer
 */
void timers_wdt_enable(void);

/**
 * Feed watchdog timer
 */
void timers_wdt_feed(void);

/**
 * Enable BNO055 delay
 * @param delay Duration to delay for in ms
 */
void timers_enable_bno055_delay(uint32_t delay);

/**
 * Delay in ms without triggering watchdog
 * @param delayms Duration in ms to delay for
 */
void timers_safe_delay(uint32_t delayms);

/**
 * Use WDT to trigger a system reset now
 */
void timers_reset_now(void);

/**
 * Setup i2c0 timeout task
 * @param target Function to run ont timeout
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
