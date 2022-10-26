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

/**
 * Delay in microseconds
 * @param us Number of microseconds to delay for
 */
void delay_us(uint32_t us);

/**
 * Delay in milliseconds
 * @param ms Number of milliseconds to delay for
 */
void delay_ms(uint32_t ms);

/**
 * Delay in seconds
 * @param sec Number of seconds to delay for
 */
void delay_sec(uint32_t sec);


/**
 * Initialize clocks
 */
void clocks_init(void);

