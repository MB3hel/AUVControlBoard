/**
 * Clock configuration
 * 
 * @file clocks.h
 * @author Marcus Behel
 */

#pragma once

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Port name macros
#define PORT_A              0
#define PORT_B              1

// Pin function macros
#define PORT_PINFUNC_A      0
#define PORT_PINFUNC_B      1
#define PORT_PINFUNC_C      2
#define PORT_PINFUNC_D      3
#define PORT_PINFUNC_E      4
#define PORT_PINFUNC_F      5
#define PORT_PINFUNC_G      6
#define PORT_PINFUNC_H      7
#define PORT_PINFUNC_I      8
#define PORT_PINFUNC_J      9
#define PORT_PINFUNC_K      10
#define PORT_PINFUNC_L      11
#define PORT_PINFUNC_M      12
#define PORT_PINFUNC_N      13

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

void ports_init(void);
