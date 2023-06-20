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


// IMPLEMENTED USING SAMD51's NVMCTRL SmartEEPROM (EEPROM emulated using flash)
// Configured as a 1024 byte EEPROM (see Table 25-6 on page 597 of datasheet)
// SBLK = 1, PSZ = 8 bytes = 1 (see table 25-11 on page 601 for PSZ values)


#include <eeprom.h>
#include <framework.h>

void eeprom_init(void){
    // TODO: Load User page from NVMCTRL
    // TODO: If SmartEEPROM bits are not correct, update the User page
    //       See https://github.com/GMagician/SAMD51-SmartEEprom-Manager/blob/master/SEEManager/SEEManager.ino
    //       This is necessary to update the device configuration bits (sometimes called fuses)
    // TODO: If this is changed, reset to apply the changes
    // TODO: If no change required, can continue with init as usual
}

void eeprom_write(uint16_t address, uint8_t *data, unsigned int len){

}

void eeprom_read(uint16_t *address, uint8_t *data, unsigned int len){

}
