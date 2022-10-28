/**
 * @file flags.h
 * Globally accessible flags to indicate when the main tree should perform
 * various actions
 * @author Marcus Behel
 */

#pragma once

#include <stdint.h>

// Macros to work with flags

// Set a flag
#define FLAG_SET(field, flag)       ((field) |= (flag))

// Clear (unset)  a flag
#define FLAG_CLEAR(field, flag)     ((field) &= ~(flag))

// Check if a flag is set
#define FLAG_CHECK(field, flag)     ((field) & (flag))


// Flags in flags_main field
#define FLAG_MAIN_10MS              0b0000000000000001                  // Set every 10ms
#define FLAG_MAIN_20MS              0b0000000000000010                  // Set every 20ms
#define FLAG_MAIN_50MS              0b0000000000000100                  // Set every 50ms
#define FLAG_MAIN_100MS             0b0000000000001000                  // Set every 100ms
#define FLAG_MAIN_1000MS            0b0000000000010000                  // Set every 1000ms
#define FLAG_MAIN_USBMSG            0b0000000000100000                  // Set when usb has message
#define FLAG_MAIN_I2C0_DONE         0b0000000001000000                  // Set when i2c0 finishes a transaction

// Defined in main.c
extern volatile uint16_t flags_main;
