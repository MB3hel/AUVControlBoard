#pragma once

#include <stdbool.h>


typedef struct {
    float pitch;
    float roll;
    float yaw;
    bool is_deg;
} euler_t;


typedef struct {
    float w;
    float x;
    float y;
    float z;
} quaternion_t;


////////////////////////////////////////////////////////////////////////////////
/// Euler operations
////////////////////////////////////////////////////////////////////////////////

/**
 * Convert euler angles from degrees to radians
 * @param dest Pointer to store radian data in
 * @param src Pointer to read degree data from
 */
void euler_deg2rad(euler_t *dest, euler_t *src);

/**
 * Convert euler angles from radians to degrees
 * @param dest Pointer to store degree data in
 * @param src Pointer to read radian data from
 */
void euler_rad2deg(euler_t *dest, euler_t *src);

/**
 * Convert euler to quaternion
 * @param dest Quaternion to store data in
 * @param src Euler angles to read and convert
 */
void euler_to_quaternion(quaternion_t *dest, euler_t *src);



////////////////////////////////////////////////////////////////////////////////
/// Quaternion operations
////////////////////////////////////////////////////////////////////////////////

// TODO
