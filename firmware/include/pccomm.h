#pragma once

#include <stdbool.h>

// Maximum message size in bytes
#define PCCOMM_MAX_MSG_LEN          96

/**
 * Read data from the PC and parse the message.
 * @return true if a complete message has been received
 */
bool pccomm_read_and_parse(void);
