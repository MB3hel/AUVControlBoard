
/**
 * @file bno055.h
 * 
 * BNO055 IMU driver using i2c0
 * 
 * @author Marcus Behel
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Axis config values
#define BNO055_AXIS_X                           0b00        // Remap to b x-axis
#define BNO055_AXIS_Y                           0b01        // Remap to be y-axis
#define BNO055_AXIS_Z                           0b10        // Remap to be z-axis
#define BNO055_AXIS_POS                         0b0         // Positive axis sign
#define BNO055_AXIS_NEG                         0b1         // Negative axis sign

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Typedefs
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct{
    // Native BNO055 axes
    // Set to desired axis
    // Eg to swap X and Z axes set x = BNO055_AXIS_Z and z = BNO055_AXIS_X
    uint8_t x, y, z;

    // Axis signs
    uint8_t sx, sy, sz;
} bno055_axis_config;

typedef struct{
    // Gravity vector (m/s^2)
    float grav_x, grav_y, grav_z;

    // Quaternion orientation (rad)
    float quat_w, quat_x, quat_y, quat_z;
} bno055_data;

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
void bno055_checki2c(void);

/**
 * Call when delay done
 */
void bno055_delay_done(void);

/**
 * Call when idle done
 */
void bno055_idle_done(void);

/**
 * Trigger a reconfigure
 */
void bno055_reconfig(bno055_axis_config new_axis_config);

/**
 * Get current data from bno055
 * 
 * @return bno055_data 
 */
bno055_data bno055_get(void);
