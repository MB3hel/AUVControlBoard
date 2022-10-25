/**
 * Clock configuration
 * 
 * @file clocks.h
 * @author Marcus Behel
 */

#pragma once

#include <stdint.h>

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

// Output directions
#define PORT_GPIO_LOW               0
#define PORT_GPIO_HIGH              1

// Port definition & manipulation macros
// Upper 3 bits for port, lower 5 for pin
#define PORT_DEFINE(port, pin)          (((port & 0b111) << 5) | (pin & 0b11111))
#define PORT_GETPORT(def)               ((def & 0b11100000) >> 5)
#define PORT_GETPIN(def)                (def & 0b00011111)

// Project port definitions
#define P_RED_LED                       PORT_DEFINE(PORT_A, 22)




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ports_pinfunc(uint8_t def, int8_t pinfunc);

void ports_gpio_dir(uint8_t def, uint8_t dir);

void ports_gpio_pull(uint8_t def, uint8_t pull);

void ports_gpio_write(uint8_t def, uint8_t out);

void ports_gpio_set(uint8_t def);

void ports_gpio_clear(uint8_t def);

void ports_gpio_toggle(uint8_t def);

uint8_t ports_gpio_read(uint8_t def);