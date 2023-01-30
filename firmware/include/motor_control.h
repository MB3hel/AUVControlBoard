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
#include <angles.h>


// Inverts positive and negative on specific thrusters
extern bool mc_invert[8];


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
 * Set motor speeds in RAW mode
 * @param speeds Array of 8 speeds (for each thruster). From -1.0 to 1.0
 */
void mc_set_raw(float *speeds);

/**
 * Set motor speeds in LOCAL mode. Target speeds are in DoF frame relative to robot, not world
 * @param x Speed in +x translation DoF (-1.0 to +1.0)
 * @param y Speed in +y translation DoF (-1.0 to +1.0)
 * @param z Speed in +z translation DoF (-1.0 to +1.0)
 * @param pitch Speed in +pitch rotation DoF (-1.o to +1.0)
 * @param roll Speed in +roll rotation DoF (-1.o to +1.0)
 * @param yaw Speed in +yaw rotation DoF (-1.o to +1.0)
 */
void mc_set_local(float x, float y, float z, float pitch, float roll, float yaw);

/**
 * Set motor speeds in GLOBAL mode. Target speeds are in DoF frame relative to world, not the robot.
 * Note that this "world frame" does not compensate for yaw (just pitch and roll)
 * @param x Speed in +x translation DoF (-1.0 to +1.0)
 * @param y Speed in +y translation DoF (-1.0 to +1.0)
 * @param z Speed in +z translation DoF (-1.0 to +1.0)
 * @param pitch Speed in +pitch rotation DoF (-1.o to +1.0)
 * @param roll Speed in +roll rotation DoF (-1.o to +1.0)
 * @param yaw Speed in +yaw rotation DoF (-1.o to +1.0)
 * @param curr_quat Current orientation quaternion from IMU
 */
void mc_set_global(float x, float y, float z, float pitch, float roll, float yaw, quaternion_t curr_quat);

/**
 * Tune stability assist mode pitch pid
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param kf Feed-forward gain
 * @param limit Magnitude of max PID output (max = limit, min = -limit). Must be positive
 */
void mc_sassist_tune_pitch(float kp, float ki, float kd, float kf, float limit);

/**
 * Tune stability assist mode roll pid
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param kf Feed-forward gain
 * @param limit Magnitude of max PID output (max = limit, min = -limit). Must be positive
 */
void mc_sassist_tune_roll(float kp, float ki, float kd, float kf, float limit);

/**
 * Tune stability assist mode yaw pid
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param kf Feed-forward gain
 * @param limit Magnitude of max PID output (max = limit, min = -limit). Must be positive
 */
void mc_sassist_tune_yaw(float kp, float ki, float kd, float kf, float limit);

/**
 * Tune stability assist mode depth pid
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param kf Feed-forward gain
 * @param limit Magnitude of max PID output (max = limit, min = -limit). Must be positive
 */
void mc_sassist_tune_depth(float kp, float ki, float kd, float kf, float limit);

/**
 * Set motor speeds in STABILITY_ASSIST mode. Abstracts a 2D plane in which the robot is controlled.
 * The other dimensions are handled by closed-loop control. 
 * Requires speeds for x and y DoFs and uses closed-loop control for depth (z), pitch, and roll. 
 * Yaw can be either a speed or handled by closed loop control.
 * Speeds are in world-relative (GLOBAL) DoFs 
 * @param x Speed in +x DoF (-1.0 to +1.0)
 * @param y Speed in +y DoF (-1.0 to +1.0)
 * @param yaw Speed in +yaw DoF (-1.0 to +1.0). Not used if use_yaw_pid is true
 * @param target_euler Target orientation (ZYX euler; yaw is ignored if use_yaw_pid is false)
 * @param curr_quat Current orientation quaternion
 * @param curr_depth Current depth in meters (negative below surface)
 * @param use_yaw_pid If true, closed loop control is used for yaw not a speed
 */
void mc_set_sassist(float x, float y, float yaw, 
        euler_t target_euler, 
        float target_depth,
        quaternion_t curr_quat,
        float curr_depth,
        bool use_yaw_pid);
