#pragma once

#include <stdbool.h>
#include <stdint.h>

// Maximum message size in bytes
#define PCCOMM_MAX_MSG_LEN          96

// Buffer to hold a message currently being read / parsed
// (size = max_len + 4 because of 2 bytes for CRC16 and 2 bytes for message ID)
extern uint8_t pccomm_read_buf[PCCOMM_MAX_MSG_LEN + 4];
extern unsigned int pccomm_read_len;

// CRC of a complete read message (only valid after read_and_process returns 1
// and becomes invalid next time read_and_process is called)
extern uint16_t pccomm_read_crc;

/**
 * Read data from the PC and parse the message.
 * @return true if a complete, valid message has been received
 */
bool pccomm_read_and_parse(void);

/**
 * Write a message to the PC (with correct format)
 * Note that write may not necessarily be instant (will be written into buffer)
 * @param msg The raw message to send (payload)
 * @param len Length of raw message
 */
void pccomm_write(uint8_t *msg, unsigned int len);
