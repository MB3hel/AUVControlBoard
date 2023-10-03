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


#include <framework.h>


#ifdef CONTROL_BOARD_V1

#include <tusb.h>

extern void SERCOM2_I2C_InterruptHandler(void);

void NMI_Handler(void){
    debug_halt(HALT_EC_FAULTIRQ);
}

void HardFault_Handler(void){
    debug_halt(HALT_EC_FAULTIRQ);
}

void MemManage_Handler(void){
    debug_halt(HALT_EC_FAULTIRQ);
}

void BusFault_Handler(void){
    debug_halt(HALT_EC_FAULTIRQ);
}

void UsageFault_Handler(void){
    debug_halt(HALT_EC_FAULTIRQ);
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

#endif // CONTROL_BOARD_V1


#ifdef CONTROL_BOARD_V2

extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim1;
extern I2C_HandleTypeDef hi2c1;

void NMI_Handler(void){
    debug_halt(HALT_EC_FAULTIRQ);
}

void HardFault_Handler(void){
    debug_halt(HALT_EC_FAULTIRQ);
}

void MemManage_Handler(void){
    debug_halt(HALT_EC_FAULTIRQ);
}

void BusFault_Handler(void){
    debug_halt(HALT_EC_FAULTIRQ);
}

void UsageFault_Handler(void){
    debug_halt(HALT_EC_FAULTIRQ);
}

void DebugMon_Handler(void){}

// Defined by FreeRTOS
// void SysTick_Handler(void){}

// Defined by FreeRTOS
// void PendSV_Handler(void){}

// Defined by FreeRTOS
// void SVC_Handler(void){}

void TIM1_TRG_COM_TIM11_IRQHandler(void){
    // HAL_TIM_IRQHandler(&htim1);
    HAL_TIM_IRQHandler(&htim11);
}

void OTG_FS_IRQHandler(void){
    tud_int_handler(0);
}

void I2C1_EV_IRQHandler(void){
    HAL_I2C_EV_IRQHandler(&hi2c1);
}

void I2C1_ER_IRQHandler(void){
    HAL_I2C_ER_IRQHandler(&hi2c1);
}


#endif // CONTROL_BOARD_V2
