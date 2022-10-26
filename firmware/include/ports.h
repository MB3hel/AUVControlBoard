/**
 * Port configuration
 * 
 * @file ports.h
 * @author Marcus Behel
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Port name macros
#define PORT_A              0
#define PORT_B              1

// Pin function macros
#define PORT_PINFUNC_GPIO           -1
#define PORT_PINFUNC_A              0
#define PORT_PINFUNC_B              1
#define PORT_PINFUNC_C              2
#define PORT_PINFUNC_D              3
#define PORT_PINFUNC_E              4
#define PORT_PINFUNC_F              5
#define PORT_PINFUNC_G              6
#define PORT_PINFUNC_H              7
#define PORT_PINFUNC_I              8
#define PORT_PINFUNC_J              9
#define PORT_PINFUNC_K              10
#define PORT_PINFUNC_L              11
#define PORT_PINFUNC_M              12
#define PORT_PINFUNC_N              13

// GPIO directions
#define PORT_GPIO_OUT               1
#define PORT_GPIO_IN                0

// Pull directions
#define PORT_GPIO_PULLOFF           0
#define PORT_GPIO_PULLUP            1
#define PORT_GPIO_PULLDOWN          2

// Output directions (same as expected bool values)
#define PORT_GPIO_LOW               0
#define PORT_GPIO_HIGH              1

// Port definition & manipulation macros
// Upper 3 bits for port, lower 5 for pin
#define PORT_DEFINE(port, pin)          (((port & 0b111) << 5) | (pin & 0b11111))
#define PORT_GETPORT(def)               ((def & 0b11100000) >> 5)
#define PORT_GETPIN(def)                (def & 0b00011111)

// -----------------------------------------------------------------------------
// Project port definitions (pins used in this program)
// -----------------------------------------------------------------------------
#define P_DS_CLK                        PORT_DEFINE(PORT_B, 2)
#define P_DS_DAT                        PORT_DEFINE(PORT_B, 3)
#define P_USB_DM                        PORT_DEFINE(PORT_A, 24)
#define P_USB_DP                        PORT_DEFINE(PORT_A, 25)
// -----------------------------------------------------------------------------


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Set function for a pin (see datasheet p32 for specific meanings per pin)
 * @param def Port / pin definition (PORT_DEF)
 * @param pinfunc Function of the pin (PORT_PINFUNC_*)
 */
void ports_pinfunc(uint8_t def, int8_t pinfunc);

/**
 * Set direction for a gpio pin
 * @param def Port / pin definition (PORT_DEF)
 * @param dir PORT_GPIO_OUT or PORT_GPIO_IN
 */
void ports_gpio_dir(uint8_t def, uint8_t dir);

/**
 * Set pullup resistor for a gpio pin
 * @param def Port / pin definition (PORT_DEF)
 * @param pull PORT_GPIO_PULLOFF, PORT_GPIO_PULLUP, or PORT_GPIO_PULLDOWN
 */
void ports_gpio_pull(uint8_t def, uint8_t pull);

/**
 * Write output of gpio pin
 * @param def Port / pin definition (PORT_DEF)
 * @param out PORT_GPIO_LOW or PORT_GPIO_HIGH
 */
void ports_gpio_write(uint8_t def, uint8_t out);

/**
 * Set output of a pin (write HIGH)
 * @param def Port / pin definition (PORT_DEF)
 */
void ports_gpio_set(uint8_t def);

/**
 * Clear output of a pin (write LOW)
 * @param def Port / pin definition (PORT_DEF)
 */
void ports_gpio_clear(uint8_t def);

/**
 * Toggle output of a pin
 * @param def Port / pin definition (PORT_DEF)
 */
void ports_gpio_toggle(uint8_t def);

/**
 * Read the value of a gpio pin
 * @param def Port / pin definition (PORT_DEF)
 * @return PORT_GPIO_LOW or PORT_GPIO_HIGH
 */
uint8_t ports_gpio_read(uint8_t def);

/**
 * Initialize ports & pins
 * Sets up initial state for each used pin
 */
void ports_init(void);