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
void euler_to_quat(quaternion_t *dest, euler_t *src);



////////////////////////////////////////////////////////////////////////////////
/// Quaternion operations
////////////////////////////////////////////////////////////////////////////////

/**
 * Multiply quaternion by a scalar
 * dest = a * b
 */
void quat_multiply_scalar(quaternion_t *dest, quaternion_t *a, float b);

/**
 * Divide quaternion by a scalar
 * dest = a / b
 */
void quat_divide_scalar(quaternion_t *dest, quaternion_t *a, float b);

/**
 * Multiply two quaternions
 * dest = a * b
 */
void quat_multiply(quaternion_t *dest, quaternion_t *a, quaternion_t *b);

/**
 * Calculate inverse of quaternion
 * dest = (src)^(-1)
 */
void quat_inverse(quaternion_t *dest, quaternion_t *src);

/**
 * Calculate conjugate of quaternion
 * dest = (src)*
 */
void quat_conjugate(quaternion_t *dest, quaternion_t *src);

/**
 * Calculate magnitude of quaternion
 * dest = |src|
 */
void quat_magnitude(float *dest, quaternion_t *src);

/**
 * Calculate dot product of two quaternions
 * dest = a • b
 */
void quat_dot(float *dest, quaternion_t *a, quaternion_t *b);

/**
 * Convert quaternion to euler
 * @param dest Euler to store data in (will be in radians)
 * @param src Quaternion angles to read and convert
 */
void quat_to_euler(euler_t *dest, quaternion_t *src);
