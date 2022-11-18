
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

    NVIC_SetPriority(USB_0_IRQn, 0UL);                              // Give USB interrupts highest priority
    NVIC_SetPriority(USB_1_IRQn, 0UL);                              // Give USB interrupts highest priority
    NVIC_SetPriority(USB_2_IRQn, 0UL);                              // Give USB interrupts highest priority
    NVIC_SetPriority(USB_3_IRQn, 0UL);                              // Give USB interrupts highest priority

    NVIC_EnableIRQ(USB_0_IRQn);                                     // Enable interrupt handlers for USB
    NVIC_EnableIRQ(USB_1_IRQn);                                     // Enable interrupt handlers for USB
    NVIC_EnableIRQ(USB_2_IRQn);                                     // Enable interrupt handlers for USB
    NVIC_EnableIRQ(USB_3_IRQn);                                     // Enable interrupt handlers for USB
#elif defined(CONTROL_BOARD_V2)
    #error "USB Not Yet Implemented for v2"
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
