/**
 * @file bno055.h
 * 
 * BNO055 IMU driver using i2c0
 * 
 * @author Marcus Behel
 */

#pragma once

#include <stdbool.h>
#include  <stdint.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Axes
#define BNO055_AXIS_X       0b00
#define BNO055_AXIS_Y       0b01
#define BNO055_AXIS_Z       0b10

// Axis signs
#define BNO055_AXIS_POS     0b0
#define BNO055_AXIS_NEG     0b1


// Construct value for axis remap
#define BNO055_AXIS_REMAP(x, y, z)      (((z & 0x3) << 4) | ((y & 0x3) << 2) | (x & 0x3))
#define BNO055_AXIS_SIGN(x, y, z)       (((x & 0x1) << 2) | ((y & 0x1) << 1) | (z & 0x1))

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Typedefs
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct {
    float grav_x, grav_y, grav_z;
    float euler_pitch, euler_roll, euler_yaw;
} bno055_data;

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

/**
 * Reconfigure IMU
 * @param new_config New configuration for the IMU
 */
void bno055_reconfig(uint8_t new_axis_remap, uint8_t new_axis_sign);

/**
 * Get BNO055 sensor data
 * 
 * @return bno055 
 */
bno055_data bno055_get(void);
