/**
 * @file bno055.h
 * 
 * BNO055 IMU driver using i2c0
 * 
 * @author Marcus Behel
 */

#pragma once

#include <stdbool.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Initialize BNO055 IMU
 */
bool bno055_init(void);

/**
 * Call when bno055 delay finishes
 */
void bno055_delay_done(void);

/**
 * Call when bno055 i2c transaction may have finished
 */
void bno055_check_i2c(void);
