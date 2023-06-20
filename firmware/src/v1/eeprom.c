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
#define SBLK_VAL    1
#define PSZ_VAL     1


#include <eeprom.h>
#include <framework.h>

static volatile uint8_t *eeprom;
static bool valid = false;

void eeprom_init(void){    
    // Check if SmartEEPROM is configured correctly (SBLK and PSZ)
    uint32_t seestat = NVMCTRL_SmartEEPROMStatusGet();
    uint8_t sblk = seestat & NVMCTRL_SEESTAT_SBLK_Msk >> NVMCTRL_SEESTAT_SBLK_Pos;
    uint8_t psz = seestat & NVMCTRL_SEESTAT_PSZ_Msk >> NVMCTRL_SEESTAT_PSZ_Pos;
    if(sblk != SBLK_VAL || psz != PSZ_VAL){
        // SBLK and PSZ are not correct. These need to be written in the user row. 
        // Then, the device is reset to apply the configuration.

        // See page 56 of datasheet for which bits are what in the user row.
        // NOTE: Must not loose other parts of user row! Must read then write! Other parts remain unchanged.
        // NOTE: Loosing power while erasing and writing this page could render the chip unusable!
        uint32_t user_row[128];
        NVMCTRL_Read(user_row, 128, NVMCTRL_USERROW_START_ADDRESS);
        user_row[1] &= ~(FUSES_USER_WORD_1_NVMCTRL_SEESBLK_Msk | FUSES_USER_WORD_1_NVMCTRL_SEEPSZ_Msk);
        user_row[1] |= FUSES_USER_WORD_1_NVMCTRL_SEESBLK(SBLK_VAL) | FUSES_USER_WORD_1_NVMCTRL_SEEPSZ(PSZ_VAL);

        // TODO: Enable this once above code has been checked under debugger.
        // TODO: Turn LED white (indication to user to not unplug)
        // NVMCTRL_USER_ROW_RowErase(NVMCTRL_USERROW_START_ADDRESS);
        // NVMCTRL_USER_ROW_PageWrite(user_row, NVMCTRL_USERROW_START_ADDRESS);
        // NVIC_SystemReset();
        // while(1);
    }

    // SmartEEPROM size is configured correctly via fuses
    // TODO: Make sure to unlock SEE (NVMCTRL command?)
    // TODO: other configuration (enable automatic reallocation, unbuffered mode)
    // TODO: assign eeprom pointer
    // TODO: Wait until SmartEEPROM not busy
    // TODO: valid = true;
}

bool eeprom_write(uint16_t address, uint8_t data){
    if(!valid)
        return false;
    while(NVMCTRL_SmartEEPROM_IsBusy());
    eeprom[address] = data;
    while(!(NVMCTRL_REGS->NVMCTRL_INTFLAG & NVMCTRL_INTFLAG_SEEWRC_Msk));
    if(NVMCTRL_REGS->NVMCTRL_INTFLAG & NVMCTRL_INTFLAG_SEESOVF_Msk){
        return false;
    }
    return true;
}

bool eeprom_read(uint16_t address, uint8_t *data){
    if(!valid)
        return false;
    while(NVMCTRL_SmartEEPROM_IsBusy());
    *data = eeprom[address];
    return true;
}
