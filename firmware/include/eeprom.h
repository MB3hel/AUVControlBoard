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

// TODO: Addresses as macros (constants)

/**
 * Initialize eeprom (or emulated eeprom)
 * Used for storing persistent data
 */
void eeprom_init(void);

/**
 * Write data to the eeprom at the specified address
 * 
 * @param address Address to write data at
 * @param data Data to be written (pointer to data array)
 * @param len Length of data to write
 */
void eeprom_write(uint16_t address, uint8_t *data, unsigned int len);

/**
 * Read data from the eeprom at the specified address
 * 
 * @param address Address to read data from
 * @param data Pointer to array to read data into
 * @param len Amount of bytes to read (data array must be at least this length)
 */
void eeprom_read(uint16_t *address, uint8_t *data, unsigned int len);
