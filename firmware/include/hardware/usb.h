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


extern bool usb_initialized;


/**
 * Initialize communication with PC via USB
 */
void usb_init(void);

/**
 * Process USB events. Blocks until at least one event handled
 * When it returns, there may be data. check with usb_avail
 * 
 */
void usb_process(void);

/**
 * @return Number of bytes available to read from USB
 */
unsigned int usb_avail(void);

/**
 * Read one byte from USB
 * @return uint8_t next byte. Return value undefined if no data available (check with usb_avail!)
 */
uint8_t usb_read(void);

/**
 * Write one byte via USB
 * @param b Byte to write
 */
void usb_write(uint8_t b);

/**
 * Flush USB output buffers now
 */
void usb_flush(void);
