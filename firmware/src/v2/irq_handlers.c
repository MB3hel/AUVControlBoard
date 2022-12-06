
#include <tusb.h>
#include <framework.h>

extern TIM_HandleTypeDef htim11;

void NMI_Handler(void){
    while(1){
        asm("nop");
    }
}

void HardFault_Handler(void){
    while(1){
        asm("nop");
    }
}

void MemManage_Handler(void){
    while(1){
        asm("nop");
    }
}

void BusFault_Handler(void){
    while(1){
        asm("nop");
    }
}

void UsageFault_Handler(void){
    while(1){
        asm("nop");
    }
}

void DebugMon_Handler(void){}

// Defined by FreeRTOS
// void SysTick_Handler(void){}

// Defined by FreeRTOS
// void PendSV_Handler(void){}

// Defined by FreeRTOS
// void SVC_Handler(void){}

void TIM1_TRG_COM_TIM11_IRQHandler(void){
    HAL_TIM_IRQHandler(&htim11);
}

void OTG_FS_IRQHandler(void){
    tud_int_handler(0);
}
