#pragma once

#include <stdint.h>

/**
 * Initialize command and control of 
 */
void cmdctrl_init(void);

/**
 * Handle a command
 * @param msg_id Identifier of the message
 * @param msg The message data
 * @param len Length of message data
 */
void cmdctrl_handle_cmd(uint16_t msg_id, uint8_t *msg, unsigned int len);
