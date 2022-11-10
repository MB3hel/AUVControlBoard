/**
 * 6 Degree of Freedom motor control math
 * See ../math/README.md for a derivation / explanation of this process 
 * as well as a definition of the robot's coordinate system
 * A maximum of 8 thrusters are supported.
 * Note that in the derivation a "motor matrix" is defined to allow motor numbers to be specified by row
 * In this implementation, motor numbers are always dof_matrix row + 1
 * meaning they always go in order from 1 (in row 0) to 8 (in row 7)
 * 
 * @file motor_control.h
 * @author Marcus Behel
 */

#pragma once

#include <stdbool.h>
#include <pid.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Array defining thruster inversions (true = inverted)
// Index = thruster number - 1
extern bool motor_control_tinv[8];


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Setup to perform motor speed calculations
 * This function will setup matrices and perform some calculations
 * to obtain results that will remain constant throughout execution
 */
void motor_control_init(void);

/**
 * Set motor speeds in "raw" mode
 * Raw mode directly specifies speeds for each motor
 * @param s1 Speed for thruster 1 (-1.0 to +1.0)
 * @param s2 Speed for thruster 1 (-1.0 to +1.0)
 * @param s3 Speed for thruster 1 (-1.0 to +1.0)
 * @param s4 Speed for thruster 1 (-1.0 to +1.0)
 * @param s5 Speed for thruster 1 (-1.0 to +1.0)
 * @param s6 Speed for thruster 1 (-1.0 to +1.0)
 * @param s7 Speed for thruster 1 (-1.0 to +1.0)
 * @param s8 Speed for thruster 1 (-1.0 to +1.0)
 */
void motor_control_raw(float s1, float s2, float s3, float s4, float s5, float s6, float s7, float s8);

/**
 * Set motor speeds in "local" mode
 * Local mode specifies target "speeds" in each of 6 DoFs relative to the robot
 * Relative to the robot meaning orientation of the robot in 3d space is ignored.
 * @param x Speed in +x translation DoF (-1.0 to +1.0)
 * @param y Speed in +y translation DoF (-1.0 to +1.0)
 * @param z Speed in +z translation DoF (-1.0 to +1.0)
 * @param pitch Speed in +pitch rotation DoF (-1.o to +1.0)
 * @param roll Speed in +roll rotation DoF (-1.o to +1.0)
 * @param yaw Speed in +yaw rotation DoF (-1.o to +1.0)
 */
void motor_control_local(float x, float y, float z, float pitch, float roll, float yaw);

/**
 * Set motor speeds in "global" mode
 * Global mode specifies target "speeds" in each of 6 DoFs relative to the WORLD (mostly)
 * Note that "yaw" in the world is ignored meaning +y is partially relative to the robot
 * Pitch and roll adjustments are world relative though. Basically, this corrects for data
 * determinable from an accelerometer
 * @param x Speed in +x translation DoF (-1.0 to +1.0)
 * @param y Speed in +y translation DoF (-1.0 to +1.0)
 * @param z Speed in +z translation DoF (-1.0 to +1.0)
 * @param pitch Speed in +pitch rotation DoF (-1.o to +1.0)
 * @param roll Speed in +roll rotation DoF (-1.o to +1.0)
 * @param yaw Speed in +yaw rotation DoF (-1.o to +1.0)
 * @param grav_x Gravity vector x component
 * @param grav_y Gravity vector y component
 * @param grav_z Gravity vector z component
 */
void motor_control_global(float x, float y, float z, float pitch, float roll, float yaw, float grav_x, float grav_y, float grav_z);

/**
 * Configure depth hold PID used in STABILITY_ASSIST mode
 */
void motor_control_cfg_depth_hold(float kp, float ki, float kd, float kf);

/**
 * Configure pitch hold PID used in STABILITY_ASSIST mode
 */
void motor_control_cfg_pitch_hold(float kp, float ki, float kd, float kf);

/**
 * Configure roll hold PID used in STABILITY_ASSIST mode
 */
void motor_control_cfg_roll_hold(float kp, float ki, float kd, float kf);

/**
 * Set motor speeds in "stability assist" mode.
 * Stability assist mode specifies target values for pitch, roll, and depth. These are 
 * achieved using tunable PID controllers. 
 * x, y, and yaw are provided as target speeds (-1.0 to 1.0) and are WORLD relative (just as in global mode)
 * Effectively, this control mode abstracts a 2D plane in which the robot operates.
 * The target depth controls the depth of this 2D plane
 * The target pitch and roll control the robot's orientation in 3D space about this plane's axes.
 * 
 * @param x Speed in x axis
 * @param y Speed in y axis
 * @param yaw Rotation speed about world z axis
 * @param pitch_target Target pitch in degrees
 * @param roll_target Target roll in degrees
 * @param depth_target Target depth in meters (negative for below surface)
 * @param curr_pitch Current pitch (deg)
 * @param curr_roll Current roll (deg)
 * @param curr_depth Current depth (m, negative for below surface)
 * @param grav_x Gravity vector x component
 * @param grav_y Gravity vector y component
 * @param grav_z Gravity vector z component
 */
void motor_control_sassit(float x, float y, float yaw, float pitch_target, float roll_target, float depth_target, float curr_pitch, float curr_roll, float curr_depth, float grav_x, float grav_y, float grav_z);

/**
 * Increments the motor watchdog by 1 count
 * Should be called every 100ms
 * 
 * Will disable motors if count has become too high
 * 
 * @return true if watchdog disabled motors
 * @return false if watchdog did not disable motors
 */
bool motor_control_watchdog_count(void);

/**
 * Feeds motor watchdog to prevent automatic disable
 */
void motor_control_watchdog_feed(void);
