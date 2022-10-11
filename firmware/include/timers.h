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
 * Set BNO055 delay flag after given delay
 */
void timers_bbo055_delay(uint32_t delay);

/**
 * Enable watchdog timer
 */
void timers_wdt_enable(void);

/**
 * Feed watchdog timer
 */
void timers_wdt_feed(void);
