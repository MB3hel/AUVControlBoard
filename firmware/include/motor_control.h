#pragma once

#include <stdint.h>
#include <stdbool.h>


// Inverts positive and negative on specific thrusters
extern bool mc_invert[8];


/**
 * Initialize motor control
 */
void mc_init(void);

/**
 * Set a row of the DoF matrix
 * Note that the math implementation does not actually use a "motor matrix" as
 * described in the math docs. Rather the DoF matrix is directly constructed.
 * @param thruster_num Thruster for which the row data is being set (1-8 NOT 0-7)
 * @param row_data Data for the given thruster's row (8 element array)
 */
void mc_set_dof_matrix(unsigned int thruster_num, float *row_data);

/**
 * Recalculate parameters after dof matrix changed
 */
void mc_recalc(void);

/**
 * Feed (reset) motor watchdog
 * @return Whether the motors were previously killed
 */
bool mc_wdog_feed(void);


/**
 * Set motor speeds in RAW mode
 * @param speeds Array of 8 speeds (for each thruster). From -1.0 to 1.0
 */
void mc_set_raw(float *speeds);

