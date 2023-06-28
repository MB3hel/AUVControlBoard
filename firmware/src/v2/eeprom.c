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

#include <eeprom.h>
#include <framework.h>
#include <led.h>
#include <st_eeprom.h>
#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>

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
