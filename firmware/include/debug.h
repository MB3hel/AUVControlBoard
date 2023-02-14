/*
 * Copyright 2023 Marcus Behel
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


// Error codes
#define HALT_EC_ASSERT          -1
#define HALT_EC_MALLOC_FAIL     -2
#define HALT_EC_SOVERFLOW       -3
#define HALT_EC_FAULTIRQ        -4
#define HALT_EC_SCHEDRET        -5



/**
 * Halt program due to some error
 * @param error_code Error code for why here
 */
void debug_halt(int error_code);

/**
 * Log a debug message to the PC via USB UART
 * @param msg String message to log
 */
void debug_log(const char *msg);

/*
 * Log a debug message to the PC (raw bytes)
 */
void debug_log_data(uint8_t *msg, unsigned int len);
