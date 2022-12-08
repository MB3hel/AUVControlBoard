#pragma once

#include <stdint.h>

/**
 * Initialize command and control of 
 */
void cmdctrl_init(void);

/**
 * Handle a message received from the pc
 */
void cmdctrl_handle_message();
