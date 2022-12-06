#include <usb.h>
#include <framework.h>
#include <tusb.h>
#include <limits.h>
#include <string.h>
#include <FreeRTOS.h>


void usb_init(void){
#if defined(CONTROL_BOARD_V1)
    // Set interrupt to highest allowed priority
    NVIC_SetPriority(USB_OTHER_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(USB_SOF_HSOF_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(USB_TRCPT0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(USB_TRCPT1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    // Pins configured by generated code
    // Clock config handled by generated code
#elif defined(CONTROL_BOARD_V2)
    // Set interrupt to highest allowed priority
    NVIC_SetPriority(OTG_FS_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    // Pins configured by generated code

    // Enable clock to USB FS peripheral
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

    // Disable VBUS sense
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;
#endif
}

void usb_device_task(void *argument){
    tud_init(BOARD_TUD_RHPORT);
    while(1){
        // This call will block thread until there is / are event(s)
        tud_task();
    }
}

void usb_write(uint8_t *data, unsigned int len){
    if(!tud_inited())
        return;
    tud_cdc_write(data, len);
    tud_cdc_write_flush();
}

void usb_writestr(const char *msg){
#if CHAR_BIT == 8
    // char is an 8-bit int, so pointer cast is fine
    // This is fast.
    usb_write((uint8_t*)msg, strlen(msg));
#else
    // Copy data. This is significantly slower.
    unsigned int len = strlen(msg);
    uint8_t *data = pvPortMalloc(len * sizeof(uint8_t));
    for(unsigned int i = 0; i < len; ++i){
        data[i] = msg[i];
    }
    usb_write(data, len);
    vPortFree(data);
#endif
}
