
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
 * Initialize communication with the BNO055. Does not configure anything.
 * 
 * @return true On success
 * @return false On failure
 */
bool bno055_init(void);

/**
 * BNO055 state machine. Starts by configuring the sensor (default settings)
 * Then enters an idle state until a reading is triggered using bno055_read
 */
void bno055_process(void);

/**
 * Start a read if in the IDLE state
 */
void bno055_read(void);
