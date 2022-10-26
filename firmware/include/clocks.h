/**
 * Clock configuration
 * 
 * @file clocks.h
 * @author Marcus Behel
 */

#pragma once

#include <stdint.h>

#define CLOCKS_GCLK_120M                    0
#define CLOCKS_GCLK_48M                     1
#define CLOCKS_GCLK_32K                     3

// SystemCoreClock in MHz
// SystemCoreClock cycles per second
// SystemCoreClock / 1000 cycles per ms
// SystemCoreClock / 1000000 cycles per us
#define CLOCKS_US_TO_CYCLES(us)             (us * (SystemCoreClock / 1000000))
#define CLOCKS_MS_TO_CYCLES(ms)             (ms * (SystemCoreClock / 1000))
#define CLOCKS_SEC_TO_CYCLES(sec)           (sec * (SystemCoreClock))

/**
 * Initialize clocks
 */
void clocks_init(void);

/**
 * Delay (busy loop) for a certain number of cycles
 * @param cycles Number of cycles to delay for
 */
void delay_cycles(uint32_t cycles);
