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
 * @param grav_x Gravity vector x component
 * @param grav_y Gravity vector y component
 * @param grav_z Gravity vector z component
 */
void mc_set_global(float x, float y, float z, float pitch, float roll, float yaw, float grav_x, float grav_y, float grav_z);

/**
 * Set motor speeds in STABILITY_ASSIST mode variant 1. Three speeds are provided for the x, y, and yaw
 * DoFs (GLOBAL reference). Other axis info is controlled by  PIDs using provided targets as setpoints.
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
void mc_set_sassist1(float x, float y, float yaw, float pitch_target, float roll_target, float depth_target, 
        float curr_pitch, float curr_roll, float curr_depth, float grav_x, float grav_y, float grav_z);

/**
 * Set motor speeds in STABILITY_ASSIST mode variant 2. Two speeds are provided for the x and y
 * DoFs (GLOBAL reference). Other axis info is controlled by  PIDs using provided targets as setpoints.
 * @param x Speed in x axis
 * @param y Speed in y axis
 * @param pitch_target Target pitch in degrees
 * @param roll_target Target roll in degrees
 * @param yaw_target Target yaw in degrees
 * @param depth_target Target depth in meters (negative for below surface)
 * @param curr_pitch Current pitch (deg)
 * @param curr_roll Current roll (deg)
 * @param curr_depth Current depth (m, negative for below surface)
 * @param grav_x Gravity vector x component
 * @param grav_y Gravity vector y component
 * @param grav_z Gravity vector z component
 */
void mc_set_sassist2(float x, float y, float pitch_target, float roll_target, float yaw_target, float depth_target, 
        float curr_pitch, float curr_roll, float curr_depth, float grav_x, float grav_y, float grav_z);
