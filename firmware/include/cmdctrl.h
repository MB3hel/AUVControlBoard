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
#include <bno055.h>
#include <ms5837.h>

/**
 * Initialize command and control of 
 */
void cmdctrl_init(void);

/**
 * Handle a message received from the pc
 */
void cmdctrl_handle_message(void);

/**
 * Reapply the last applied speed
 */
void cmdctrl_apply_saved_speed(void);

/**
 * Call when motor watchdog status changes
 * @param motors_enabled True if motors enabled, False if motors killed
 */
void cmdctrl_mwdog_change(bool motors_enabled);

/**
 * Set bno055 status
 * @param status New status (ready = true; not ready = false)
 */
void cmdctrl_bno055_status(bool status);

/**
 * Set bno055 data
 * @param data New sensor data
 */
void cmdctrl_bno055_data(bno055_data data);

/**
 * Set ms5837 status
 * @param status New status (ready = true; not ready = false)
 */
void cmdctrl_ms5837_status(bool status);

/**
 * Set ms5837 data
 * @param data New sensor data
 */
void cmdctrl_ms5837_data(ms5837_data data);

/**
 * Send SIMSTAT message
 */
void cmdctrl_send_simstat(void);
