#pragma once

#include <matrix.h>
#include <stdint.h>
#include <stdbool.h>


// Inverts positive and negative on specific thrusters
extern bool mc_invert[8];


/**
 * Initialize motor control
 */
void mc_init(void);

/**
 * Set the motor matrix defining thruster configuration
 * @param motor_mat Motor matrix (must be 8x7). Will be copied (does not need to remain valid after function returns)
 */
void mc_set_motor_matrix(matrix *motor_mat);

/**
 * Feed (reset) motor watchdog
 */
void mc_wdog_feed(void);

/**
 * Set motor speeds in RAW mode
 * @param speeds Array of 8 speeds (for each thruster). From -1.0 to 1.0
 */
void mc_set_raw(float *speeds);

