/**
 * @file cmdctrl.h
 * 
 * Command and control interface for control board. Handles commands received from the pc
 * and facilitates transfering data from control board to the pc.
 * 
 * @author Marcus Behel
 */

#pragma once

#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Initialize command & control interface
 */
void cmdctrl_init(void);

/**
 * Handle a message from the pc
 */
void cmdctrl_handle_msg(uint8_t *msg, uint32_t len);

/**
 * Set RGB LED color based on mode
 */
void cmdctrl_update_led(void);

/**
 * Inform the PC that motor watchdog killed motors
 */
void cmdctrl_motors_killed(void);

/**
 * Send sensor data to PC
 */
void cmdctrl_send_sensors(void);

/**
 * Update motors (periodic) if needed in the current mode of operation
 */
void cmdctrl_update_motors(void);
