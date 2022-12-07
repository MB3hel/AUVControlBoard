#pragma once


/**
 * Initialize thruster control
 */
void thruster_init(void);

/**
 * Set thruster speeds
 * @param speeds An array of 8 floats between -1.0 and 1.0
 *               Each corresponds to a thruster speed for thruster index+1
 */
void thruster_set(float *speeds);
