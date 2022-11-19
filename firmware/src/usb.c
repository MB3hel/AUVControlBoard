
#include <usb.h>
#include <framework.h>
#include <tusb.h>


void usb_init(void){
#if defined(CONTROL_BOARD_V1)
    gpio_set_pin_function(GPIO(GPIO_PORTA, 24), GPIO_PIN_FUNCTION_H);
    gpio_set_pin_function(GPIO(GPIO_PORTA, 25), GPIO_PIN_FUNCTION_H);

    GCLK->PCHCTRL[USB_GCLK_ID].bit.GEN = 1;                         // Select 48MHz clock for USB
    GCLK->PCHCTRL[USB_GCLK_ID].bit.CHEN = 1;                        // Enable clock to USB
    MCLK->AHBMASK.bit.USB_ = 1;                                     // Enable AHB clock to USB
    MCLK->APBBMASK.bit.USB_ = 1;                                    // Enable APB clock to USB

    NVIC_SetPriority(USB_0_IRQn, 
            configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );         // Give USB interrupts highest priority
    NVIC_SetPriority(USB_1_IRQn, 
            configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );         // Give USB interrupts highest priority
    NVIC_SetPriority(USB_2_IRQn, 
            configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );         // Give USB interrupts highest priority
    NVIC_SetPriority(USB_3_IRQn, 
            configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );         // Give USB interrupts highest priority

    NVIC_EnableIRQ(USB_0_IRQn);                                     // Enable interrupt handlers for USB
    NVIC_EnableIRQ(USB_1_IRQn);                                     // Enable interrupt handlers for USB
    NVIC_EnableIRQ(USB_2_IRQn);                                     // Enable interrupt handlers for USB
    NVIC_EnableIRQ(USB_3_IRQn);                                     // Enable interrupt handlers for USB
#elif defined(CONTROL_BOARD_V2)    
    NVIC_SetPriority(OTG_FS_IRQn, 
            configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );         // Give USB FS interrupt highest priority
    NVIC_EnableIRQ(OTG_FS_IRQn);                                    // Enable USB FS interrupt handler

    // Enable USB OTG clock
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;
#endif
    tud_init(BOARD_TUD_RHPORT);                                     // Initialize TinyUSB device
}

void usb_process(void){
    tud_task();
}

void usb_write(const char *msg){
    tud_cdc_write(msg, strlen(msg));
    tud_cdc_write_flush();
}
