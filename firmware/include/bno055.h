#pragma once

#include <stdbool.h>
#include <stdint.h>

// Axis configurations
// Note: Px macro needs to have value x
#define BNO055_AXIS_P0          0
#define BNO055_AXIS_P1          1
#define BNO055_AXIS_P2          2
#define BNO055_AXIS_P3          3
#define BNO055_AXIS_P4          4
#define BNO055_AXIS_P5          5
#define BNO055_AXIS_P6          6
#define BNO055_AXIS_P7          7

typedef struct{
    float grav_x, grav_y, grav_z;
    float euler_pitch, euler_roll, euler_yaw;
} bno055_data;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// BNO055 Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Initialize BNO055 driver
 */
void bno055_init(void);

/**
 * Configure the sensor
 * 
 * @return true on success; false on error
 */
bool bno055_configure(void);

/**
 * Set axis remap and sign configuration. Must be configured before running
 * 
 * @param mode Mode to set (BNO055_AXIS_Px)
 * @return true on success; false on error
 */
bool bno055_set_axis(uint8_t mode);

/**
 * Read data from IMU. Must be configured before running
 * 
 * @param data Pointer to struct to store data in (only valid if returns true)
 * @return true On success; false on error
 */
bool bno055_read(bno055_data *data);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

