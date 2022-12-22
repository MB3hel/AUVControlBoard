#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <bno055.h>

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
