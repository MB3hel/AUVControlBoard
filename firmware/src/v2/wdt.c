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

#include <wdt.h>
#include <framework.h>

extern IWDG_HandleTypeDef hiwdg;

extern void MX_IWDG_Init(void);

void wdt_init(void){
    // Init handled in CubeMX generated code
    // LSI = 32kHz on this board
    // Prescaler = 32  ->  1024 counts / second
    // Reload configured to 2048 ~= 2 second timeout
    MX_IWDG_Init();
    
    // Freeze watchdog when core halted
    __HAL_DBGMCU_FREEZE_IWDG();
}

void wdt_feed(void){
    HAL_IWDG_Refresh(&hiwdg);
}

void wdt_reset_now(void){
    // Doesn't actually use WDT on this chip, but function name kept for compatability
    NVIC_SystemReset();
    while(1);
}
