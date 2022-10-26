/**
 * SW "SPI" interface to control RGB "dotstar" LED on ItsyBitsy M4
 * Note: This is not a suitable library for strips of LEDs
 * Setting colors blocks
 * @file dotstar.h
 * @author Marcus Behel
 */
#pragma once

#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Initialize dotstar sw library
 */
void dotstar_init(void);

/**
 * Set led color (BLOCKS WHILE WRITING)
 * @param r Red component
 * @param g Green component
 * @param b Blue component
 */
void dotstar_set(uint8_t r, uint8_t g, uint8_t b);
