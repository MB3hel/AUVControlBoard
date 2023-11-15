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


#include <hardware/eeprom.h>
#include <framework.h>


#ifdef CONTROL_BOARD_V1

#include <hardware/led.h>

// IMPLEMENTED USING SAMD51's NVMCTRL SmartEEPROM (EEPROM emulated using flash)
// Configured as a 1024 byte EEPROM (see Table 25-6 on page 597 of datasheet)
// SBLK = 1, PSZ = 8 bytes = 1 (see table 25-11 on page 601 for PSZ values)
// NOTE: If this is changed you must edit the linker script too (see table 25-10 on page 600 of datasheet)
#define SBLK_VAL    1
#define PSZ_VAL     1

static volatile uint16_t *eeprom;
static bool valid = false;

void eeprom_init(void){    
    // Check if SmartEEPROM is configured correctly (SBLK and PSZ)
    uint32_t seestat = NVMCTRL_SmartEEPROMStatusGet();
    uint32_t sblk = (seestat & NVMCTRL_SEESTAT_SBLK_Msk) >> NVMCTRL_SEESTAT_SBLK_Pos;
    uint32_t psz = (seestat & NVMCTRL_SEESTAT_PSZ_Msk) >> NVMCTRL_SEESTAT_PSZ_Pos;
    if(sblk != SBLK_VAL || psz != PSZ_VAL){
        // SBLK and PSZ are not correct. These need to be written in the user row. 
        // Then, the device is reset to apply the configuration.

        // See page 56 of datasheet for which bits are what in the user row.
        // NOTE: Must not lose other parts of user row! Must read then write! Other parts remain unchanged.
        // NOTE: Losing power while erasing and writing this page could render the chip unusable!
        uint32_t user_row[128];
        NVMCTRL_Read(user_row, 128, NVMCTRL_USERROW_START_ADDRESS);
        user_row[1] &= ~(FUSES_USER_WORD_1_NVMCTRL_SEESBLK_Msk | FUSES_USER_WORD_1_NVMCTRL_SEEPSZ_Msk);
        user_row[1] |= FUSES_USER_WORD_1_NVMCTRL_SEESBLK(SBLK_VAL) | FUSES_USER_WORD_1_NVMCTRL_SEEPSZ(PSZ_VAL);

        // NOTE: LED is white to indicate to user that poweroff is not safe!
        led_set(255, 255, 255);
        NVMCTRL_USER_ROW_RowErase(NVMCTRL_USERROW_START_ADDRESS);
        NVMCTRL_USER_ROW_PageWrite(user_row, NVMCTRL_USERROW_START_ADDRESS);
        NVIC_SystemReset();
        while(1);
    }

    // SmartEEPROM size is configured correctly via fuses

    // Unlock SmartEEPROM if locked
    if(NVMCTRL_REGS->NVMCTRL_SEESTAT & NVMCTRL_SEESTAT_LOCK_Msk){
        NVMCTRL_REGS->NVMCTRL_CTRLB = NVMCTRL_CTRLB_CMD_USEE | NVMCTRL_CTRLB_CMDEX_KEY;
    }

    // Configure SmartEEPROM (unbuffered, automatic reallocation)
    NVMCTRL_REGS->NVMCTRL_SEECFG = NVMCTRL_SEECFG_APRDIS(0) | NVMCTRL_SEECFG_WMODE(0);

    // Assign pointer and flag valid once ready
    eeprom = (uint16_t*)(SEEPROM_ADDR);
    while(NVMCTRL_SmartEEPROM_IsBusy());
    valid = true;
}

bool eeprom_write(uint16_t address, uint16_t data){
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

bool eeprom_read(uint16_t address, uint16_t *data){
    if(!valid)
        return false;
    while(NVMCTRL_SmartEEPROM_IsBusy());
    *data = eeprom[address];
    return true;
}

#endif // CONTROL_BOARD_V1


#ifdef CONTROL_BOARD_V2

#include <st_eeprom.h>

static bool valid = false;
uint16_t VirtAddVarTab[NB_OF_VAR];

void eeprom_init(void){  
    for(unsigned int VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++){
        // Start address of 0x000A is arbitrary. Just can't be 0xFFFF and maybe not 0x0000 (unsure about 0x0000)
        // 0x000A was used during debugging, and forgot to change it back after some use, so just keeping it
        VirtAddVarTab[VarIndex] = 0x000A + VarIndex;
    } 
    HAL_FLASH_Unlock();
    uint16_t res = EE_Init();
    HAL_FLASH_Lock();
    if(res == EE_OK){
        valid = true;
    }
}

bool eeprom_write(uint16_t address, uint16_t data){
    if(!valid)
        return false;
    if(address >= NB_OF_VAR)
        return false;
    HAL_FLASH_Unlock();
    // Without clearing these flags, the first write usually fails
    __HAL_FLASH_CLEAR_FLAG((FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR | FLASH_FLAG_RDERR));
    uint16_t res = EE_WriteVariable(VirtAddVarTab[address], data);
    HAL_FLASH_Lock();
    return res == EE_OK;
}

bool eeprom_read(uint16_t address, uint16_t *data){
    if(!valid)
        return false;
    if(address >= NB_OF_VAR)
        return false;
    uint16_t res = EE_ReadVariable(VirtAddVarTab[address], data);
    if(res == EE_OK){
        // data is already set by EE_ReadVariable
        return true;
    }else{
        // This occurs if variable does not exist in eeprom
        // In this case, we will treat it as if the variable's value is what the 
        // flash would hold if it were erased
        // This ensures that the variable pointed to by data is in fact set
        *data = 0xFFFF;

        // We consider all reads to succeed (even if the variable is not in emulated eeprom structure)
        // This ensures consistent behavior with v1 SmartEEPROM or a physical eeprom (you can always read an address)
        return true;
    }
}

#endif // CONTROL_BOARD_V2


#ifdef CONTROL_BOARD_SIM

// Eeprom not supported for simcb, so just implement dummy functions that always fail

void eeprom_init(void){
    // Nothing needed here
}

bool eeprom_write(uint16_t address, uint16_t data){
    // Not implemented in simcb
    return false;
}

bool eeprom_read(uint16_t address, uint16_t *data){
    // Not implemented in simcb
    return false;
}

#endif // CONTROL_BOARD_SIM
