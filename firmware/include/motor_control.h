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
