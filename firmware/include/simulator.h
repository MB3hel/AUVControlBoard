/*
 * Copyright 2023 Marcus Behel
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
#include <bno055.h>
#include <ms5837.h>


// Controls whether simulation mode operation
// When in simulation mode (sim_hijacked = true)
//   - IMU and depth sensor continue to be read, but the data is unused
//   - cmdctrl treats all sensors as connected
//   - cmdctrl uses sim_quat and sim_depth instead of real sensor data
//   - Motor control calculations (motor_control.c) are all performed (including RAW mode)
//   - Motor control local mode results are cached in the variables below
//   - Thruster speed sets are disabled (PWM signals will maintain zero speed signal)
extern bool sim_hijacked;

// Data provided by the simulator (SIMDAT command to control board)
extern bno055_data sim_bno055;
extern ms5837_data sim_ms5837;

// Data provided to the simulator (SIMSTAT command from control board)
extern float sim_local_x;
extern float sim_local_y;
extern float sim_local_z;
extern float sim_local_pitch;
extern float sim_local_roll;
extern float sim_local_yaw;
// mode is provided too, but tracked in cmdctrl
// wdog_killed is provided too, but tracked in cmdctrl
