/**
 * Timer configuration
 * 
 * @file timers.h
 * @author Marcus Behel
 */

#pragma once


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
