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
#include <bno055.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define CMDCTRL_MODE_RAW            0
#define CMDCTRL_MODE_LOCAL          1


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
 * Call when motors are killed by watchdog
 */
void cmdctrl_motors_killed(void);

/**
 * Get current operating mode
 * @return CMD_CTRL_MODE_...
 */
uint8_t cmdctrl_get_mode(void);

/**
 * Send IMU data from BNO055 to PC
 */
void cmdctrl_send_bno055(bno055_data data);
