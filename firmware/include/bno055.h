#pragma once

#include <stdbool.h>
#include <stdint.h>

// Axis configurations
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
    // TODO: Euler, Quaternion, etc
} bno055_data;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// BNO055 Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void bno055_init(void);

bool bno055_configure(void);

bool bno055_set_axis(uint8_t mode);

bool bno055_read(bno055_data *data);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

