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
#define FLAG_MAIN_BNO055_DELAY      0b0000000010000000                  // Set when bno055 delay done
#define FLAG_MAIN_I2C0_TIMEOUT      0b0000000100000000                  // Set when i2c0 timeout done
#define FLAG_MAIN_BNO055_WANTI2C    0b0000001000000000                  // Set when bno055 wants i2c bus
#define FLAG_MAIN_MS5837_DELAY      0b0000010000000000                  // Set when ms5837 delay done
#define FLAG_MAIN_MS5837_WANTI2C    0b0000100000000000                  // Set when ms5837 wants i2c bus


// Defined in main.c
extern volatile uint16_t flags_main;
