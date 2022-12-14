
#include <tusb.h>
#include <framework.h>
#include <led.h>


extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim1;
extern I2C_HandleTypeDef hi2c1;

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
