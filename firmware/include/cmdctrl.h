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
#include <sensor/bno055.h>
#include <sensor/ms5837.h>


// Controls whether simulation mode operation
// When in simulation mode (cmdctrl_sim_hijacked = true)
//   - IMU and depth sensor continue to be read, but the data is unused
//   - cmdctrl treats all sensors as connected
//   - cmdctrl uses cmdctrl_sim_quat and cmdctrl_sim_depth instead of real sensor data
//   - Motor control calculations (motor_control.c) are all performed (including RAW mode)
//   - Motor control local mode results are cached in the variables below
//   - Thruster speed sets are disabled (PWM signals will maintain zero speed signal)
extern bool cmdctrl_sim_hijacked;

// Data provided by the simulator (SIMDAT command to control board)
extern quaternion_t cmdctrl_sim_quat;
extern float cmdctrl_sim_depth;

// Data provided to the simulator (SIMSTAT command from control board)
extern float cmdctrl_sim_speeds[8];
// mode is provided too, but tracked in cmdctrl
// wdog_killed is provided too, but tracked in cmdctrl



/**
 * Initialize command and control of 
 */
void cmdctrl_init(void);

/**
 * Handle a message received from the pc
 */
void cmdctrl_handle_message(void);

/**
 * Send motor watchdog status message
 * @param motors_enabled True if motors enabled, False if motors killed
 */
void cmdctrl_send_mwodg_status(bool motors_enabled);

/**
 * Send SIMSTAT message
 */
void cmdctrl_send_simstat(void);

/**
 * Configure simulator hijack state
 * Generally, this is only called internally (but USB disconnect may trigger)
 */
void cmdctrl_simhijack(bool hijack);

/**
 * Send heartbeat message
 */
void cmdctrl_send_heartbeat(void);
