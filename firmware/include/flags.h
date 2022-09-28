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
#define FLAG_MAIN_10MS              0b00000001      // 10ms elapsed
#define FLAG_MAIN_100MS             0b00000010      // 100ms elapsed
#define FLAG_MAIN_1000MS            0b00000100      // 1000ms elapsed
#define FLAG_MAIN_PCCOMM_PROC       0b00001000      // pccomm needs process
#define FLAG_MAIN_PCCOMM_MSG        0b00010000      // pccomm has a message

// Defined in main.c
extern uint8_t flags_main;

