/*
 * Copyright 2022 Marcus Behel
 * 
 * This file is part of AUVControlBoard-Firmware.
 * 
 * AUVControlBoard-Firmware is free software: you can redistribute it and/or modify it under the terms of the GNU 
 * General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 * 
 * AUVControlBoard-Firmware is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even 
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for 
 * more details.
 * 
 * You should have received a copy of the GNU General Public License along with AUVControlBoard-Firmware. If not, see 
 * <https://www.gnu.org/licenses/>. 
 * 
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <util/angles.h>


// Inverts positive and negative on specific thrusters
extern bool mc_invert[8];

// Relative scale down factors for motions
// Calculated from data provided in RELDOF command
// Contains 2 groups: linear = [x, y, z] = first half of array
//                    angular = [xrot, yrot, zrot] = second half of array
// These are multiplied by speeds to scale DOWN faster motors as needed
// Scaling down ensures that all speeds will be less than 1.0 if they started that way
extern float mc_relscale[6];


/**
 * Initialize motor control
 */
void mc_init(void);

/**
 * Set a row of the DoF matrix
 * Note that the math implementation does not actually use a "motor matrix" as
 * described in the math docs. Rather the DoF matrix is directly constructed.
 * @param thruster_num Thruster for which the row data is being set (1-8 NOT 0-7)
 * @param row_data Data for the given thruster's row (6 element array)
 */
void mc_set_dof_matrix(unsigned int thruster_num, float *row_data);

/**
 * Recalculate parameters after dof matrix changed
 */
void mc_recalc(void);

/**
 * Feed (reset) motor watchdog
 * @return Whether the motors were previously killed
 */
bool mc_wdog_feed(void);

/**
 * Tune stability assist mode x rotation pid
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param limit Magnitude of max PID output (max = limit, min = -limit). Must be positive
 * @param invert True to negate PID output
 */
void mc_sassist_tune_xrot(float kp, float ki, float kd, float limit, bool invert);

/**
 * Tune stability assist mode y rotation pid
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param limit Magnitude of max PID output (max = limit, min = -limit). Must be positive
 * @param invert True to negate PID output
 */
void mc_sassist_tune_yrot(float kp, float ki, float kd, float limit, bool invert);

/**
 * Tune stability assist mode z rotation pid
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param limit Magnitude of max PID output (max = limit, min = -limit). Must be positive
 * @param invert True to negate PID output
 */
void mc_sassist_tune_zrot(float kp, float ki, float kd, float limit, bool invert);

/**
 * Tune stability assist mode depth pid
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param limit Magnitude of max PID output (max = limit, min = -limit). Must be positive
 * @param invert True to negate PID output
 */
void mc_sassist_tune_depth(float kp, float ki, float kd, float limit, bool invert);

/**
 * Set motor speeds in RAW mode
 * @param speeds Array of 8 speeds (for each thruster). From -1.0 to 1.0
 */
void mc_set_raw(float *speeds);

/**
 * Set motor speeds in LOCAL mode. Target speeds are in DoFs relative to robot, not world
 * @param x Speed in +x translation DoF (-1.0 to +1.0)
 * @param y Speed in +y translation DoF (-1.0 to +1.0)
 * @param z Speed in +z translation DoF (-1.0 to +1.0)
 * @param xrot Angular speed about x (-1.o to +1.0)
 * @param yrot Angular speed about y (-1.o to +1.0)
 * @param zrot Angular speed about z (-1.o to +1.0)
 */
void mc_set_local(float x, float y, float z, float xrot, float yrot, float zrot);

/**
 * Set motor speeds in GLOBAL mode.
 * x, y, and z DoFs are pitch and roll compensated
 * @param x Speed in +x translation DoF (-1.0 to +1.0)
 * @param y Speed in +y translation DoF (-1.0 to +1.0)
 * @param z Speed in +z translation DoF (-1.0 to +1.0)
 * @param pitch_spd Rate of change of vehicle pitch (-1.o to +1.0)
 * @param roll_spd Rate of change of vehicle roll (-1.o to +1.0)
 * @param yaw_spd Rate of change of vehicle yaw (-1.o to +1.0)
 * @param curr_quat Current orientation quaternion from IMU
 */
void mc_set_global(float x, float y, float z, float pitch_spd, float roll_spd, float yaw_spd, quaternion_t curr_quat);

/**
 * Set motor speeds in STABILITY_ASSIST mode. Abstracts a 2D plane in which the robot is controlled.
 * The other dimensions are handled by closed-loop control. 
 * Requires speeds for x and y DoFs and uses closed-loop control for depth (z), pitch, and roll. 
 * Yaw can be adjusted by speed or by using closed-loop control
 * x & y speeds are pitch and roll compensated
 * @param x Speed in +x DoF (-1.0 to +1.0)
 * @param y Speed in +y DoF (-1.0 to +1.0)
 * @param yaw_spd Rate of change of vehicle yaw (-1.0 to 1.0)
 * @param target_euler Target orientation (ZYX euler; yaw is ignored if use_yaw_pid is false)
 * @param target_depth Target vehicle depth (meters, negative is below surface)
 * @param curr_quat Current orientation quaternion
 * @param curr_depth Current depth in meters (negative below surface)
 * @param use_yaw_pid If true, closed loop control is used for yaw not a speed
 */
void mc_set_sassist(float x, float y, float yaw_spd, 
        euler_t target_euler, 
        float target_depth,
        quaternion_t curr_quat,
        float curr_depth,
        bool use_yaw_pid);

/**
 * Set motor speeds in DEPTH_HOLD mode. Basically just GLOBAL mode, but instead of a speed
 * for the z axis, the SASSIST PID is used instead.
 * x and y DoFs are pitch and roll compensated
 * @param x Speed in +x translation DoF (-1.0 to +1.0)
 * @param y Speed in +y translation DoF (-1.0 to +1.0)
 * @param pitch_spd Rate of change of vehicle pitch (-1.0 to +1.0)
 * @param roll_spd Rate of change of vehicle roll (-1.0 to +1.0)
 * @param yaw_spd Rate of change of vehicle yaw (-1.0 to +1.0)
 * @param target_depth Desired depth in meters (negative for below surface)
 * @param curr_quat Current orientation quaternion from IMU
 * @param curr_depth Current depth in meters
 */
void mc_set_dhold(float x, float y, float pitch_spd, float roll_spd, float yaw_spd, float target_depth, quaternion_t curr_quat, float curr_depth);


/**
 * Set motor speeds in ORIENTATION_HOLD mode. Like SASSIST, but a speed is provided by depth
 * instead of depth being controlled by the PID.
 * Abstracts a 2D plane in which the robot is controlled.
 * Like SASSIST, has two variants (with and without yaw PID control)
 * @param x Speed in +x DoF (-1.0 to +1.0)
 * @param y Speed in +y DoF (-1.0 to +1.0)
 * @param z Speed in +z DoF (-1.0 to 1.0)
 * @param yaw_spd Rate of change of vehicle yaw (-1.0 to 1.0)
 * @param target_euler Target orientation (ZYX euler; yaw is ignored if use_yaw_pid is false)
 * @param curr_quat Current orientation quaternion
 * @param use_yaw_pid If true, closed loop control is used for yaw not a speed
 */
void mc_set_ohold(float x, float y, float z, float yaw_spd, 
        euler_t target_euler, 
        quaternion_t curr_quat,
        bool use_yaw_pid);
