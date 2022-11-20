
#include <usb.h>
#include <framework.h>
#include <tusb.h>


void usb_init(void){
#if defined(CONTROL_BOARD_V1)
    // Pin functions and clock configured in MCC project

    NVIC_SetPriority(USB_OTHER_IRQn, 
            configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );         // Give USB interrupts highest priority
    NVIC_SetPriority(USB_SOF_HSOF_IRQn, 
            configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );         // Give USB interrupts highest priority
    NVIC_SetPriority(USB_TRCPT0_IRQn, 
            configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );         // Give USB interrupts highest priority
    NVIC_SetPriority(USB_TRCPT1_IRQn, 
            configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );         // Give USB interrupts highest priority

    NVIC_EnableIRQ(USB_OTHER_IRQn);                                 // Enable interrupt handlers for USB
    NVIC_EnableIRQ(USB_SOF_HSOF_IRQn);                              // Enable interrupt handlers for USB
    NVIC_EnableIRQ(USB_TRCPT0_IRQn);                                // Enable interrupt handlers for USB
    NVIC_EnableIRQ(USB_TRCPT1_IRQn);                                // Enable interrupt handlers for USB
#elif defined(CONTROL_BOARD_V2)  
    // Pin functions and clock configured in STM32CubeMX project

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
