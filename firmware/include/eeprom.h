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
#include <stdbool.h>

/**
 * Initialize eeprom (or emulated eeprom)
 * Used for storing persistent data
 */
void eeprom_init(void);

/**
 * Write data to the eeprom at the specified address
 * 
 * @param address Address to write data at (0 for first byte of eeprom)
 * @param data Data to be written 
 */
bool eeprom_write(uint16_t address, uint8_t data);

/**
 * Read data from the eeprom at the specified address
 * 
 * @param address Address to read data from (0 for first byte of eeprom)
 * @param data Pointer to variable to store read data in
 */
bool eeprom_read(uint16_t address, uint8_t *data);
