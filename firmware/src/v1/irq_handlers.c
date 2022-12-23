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


#include <tusb.h>
#include <framework.h>
#include <led.h>

extern void SERCOM2_I2C_InterruptHandler(void);

void NMI_Handler(void){
    taskDISABLE_INTERRUPTS();
    led_set(255, 0, 0);
    while(1){
        asm("nop");
    }
}

void HardFault_Handler(void){
    taskDISABLE_INTERRUPTS();
    led_set(255, 0, 0);
    while(1){
        asm("nop");
    }
}

void MemManage_Handler(void){
    taskDISABLE_INTERRUPTS();
    led_set(255, 0, 0);
    while(1){
        asm("nop");
    }
}

void BusFault_Handler(void){
    taskDISABLE_INTERRUPTS();
    led_set(255, 0, 0);
    while(1){
        asm("nop");
    }
}

void UsageFault_Handler(void){
    taskDISABLE_INTERRUPTS();
    led_set(255, 0, 0);
    while(1){
        asm("nop");
    }
}

void DebugMon_Handler(void){}

// Defined by FreeRTOS
// void SysTick_Handler(void){}

// Defined by FreeRTOS
// void SVCall_Handler(void){}

// Defined by FreeRTOS
// void PendSV_Handler(void){}

void USB_OTHER_Handler (void){
    tud_int_handler(0);
}

void USB_SOF_HSOF_Handler (void){
    tud_int_handler(0);
}

void USB_TRCPT0_Handler (void){
    tud_int_handler(0);
}

void USB_TRCPT1_Handler (void){
    tud_int_handler(0);
}

void SERCOM2_0_Handler(void){
    SERCOM2_I2C_InterruptHandler();
}

void SERCOM2_1_Handler(void){
    SERCOM2_I2C_InterruptHandler();
}

void SERCOM2_2_Handler(void){
    SERCOM2_I2C_InterruptHandler();
}

void SERCOM2_OTHER_Handler(void){
    SERCOM2_I2C_InterruptHandler();
}