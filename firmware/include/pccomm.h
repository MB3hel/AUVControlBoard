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
 * Initialize data structures used to communicate with PC
 */
void pccomm_init(void);

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
