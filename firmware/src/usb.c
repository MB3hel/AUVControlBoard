
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
    GPIO_InitTypeDef  GPIO_InitStruct;

    NVIC_SetPriority(OTG_FS_IRQn, 
            configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );         // Give USB FS interrupt highest priority
    NVIC_EnableIRQ(OTG_FS_IRQn);                                    // Enable USB FS interrupt handler

    /* Configure USB FS GPIOs */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure USB D+ D- Pins */
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure VBUS Pin */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ID Pin */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Enable USB OTG clock
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

    //  __HAL_RCC_USB_OTG_HS_CLK_ENABLE();

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
