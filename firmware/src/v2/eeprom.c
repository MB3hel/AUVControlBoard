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
    for(unsigned int VarIndex = 1; VarIndex <= NB_OF_VAR; VarIndex++){
        VirtAddVarTab[VarIndex-1] = VarIndex;
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
    HAL_FLASH_Unlock();
    uint16_t res = EE_WriteVariable(VirtAddVarTab[address], data);
    HAL_FLASH_Lock();
    return res == EE_OK;
}

bool eeprom_read(uint16_t address, uint16_t *data){
    if(!valid)
        return false;
    uint16_t res = EE_ReadVariable(VirtAddVarTab[address], data);
    return res == EE_OK;
}
