
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