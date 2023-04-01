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

void wdt_init(void){
    WDT_TimeoutPeriodSet(0x8);          // Configure to timeout after 2048 cycles (1024 Hz clock) ~ 2sec
    WDT_DisableWindowMode();
    WDT_Enable();
}

void wdt_feed(void){
    WDT_Clear();
}

void wdt_reset_now(void){
    // Doesn't actually use WDT anymore on this chip, but function name kept for compatibility
    NVIC_SystemReset();
    while(1);
    // while(WDT_REGS->WDT_SYNCBUSY != 0U){}
    // WDT_REGS->WDT_CLEAR = 0x00;     // Write anything but 0xA5 resets now
}
