
#include <pccomm.h>

#include <sam.h>
#include <clocks.h>

#include "tusb.h"


void pccomm_init(void){
    GCLK->PCHCTRL[USB_GCLK_ID].bit.GEN = CLOCKS_GCLK_48M;
    GCLK->PCHCTRL[USB_GCLK_ID].bit.CHEN = 1;
    
    MCLK->AHBMASK.bit.USB_ = 1;
    MCLK->APBBMASK.bit.USB_ = 1;

	// Set up the USB DP/DN pins
    PORT->Group[0].PINCFG[PIN_PA24H_USB_DM].bit.PMUXEN = 1;
    PORT->Group[0].PMUX[PIN_PA24H_USB_DM / 2].reg &=
        ~(0xF << (4 * (PIN_PA24H_USB_DM & 0x01u)));
    PORT->Group[0].PMUX[PIN_PA24H_USB_DM / 2].reg |=
        MUX_PA24H_USB_DM << (4 * (PIN_PA24H_USB_DM & 0x01u));
    PORT->Group[0].PINCFG[PIN_PA25H_USB_DP].bit.PMUXEN = 1;
    PORT->Group[0].PMUX[PIN_PA25H_USB_DP / 2].reg &=
        ~(0xF << (4 * (PIN_PA25H_USB_DP & 0x01u)));
    PORT->Group[0].PMUX[PIN_PA25H_USB_DP / 2].reg |=
        MUX_PA25H_USB_DP << (4 * (PIN_PA25H_USB_DP & 0x01u));

	NVIC_SetPriority(USB_0_IRQn, 0UL);
	NVIC_SetPriority(USB_1_IRQn, 0UL);
	NVIC_SetPriority(USB_2_IRQn, 0UL);
	NVIC_SetPriority(USB_3_IRQn, 0UL);

    NVIC_EnableIRQ(USB_0_IRQn);
    NVIC_EnableIRQ(USB_1_IRQn);
    NVIC_EnableIRQ(USB_2_IRQn);
    NVIC_EnableIRQ(USB_3_IRQn);

	tusb_init();
}

void pccomm_process(void){
    tud_task();
}

void USB_0_Handler (void){
    tud_int_handler(0);
}

void USB_1_Handler (void){
    tud_int_handler(0);
}

void USB_2_Handler (void){
    tud_int_handler(0);
}

void USB_3_Handler (void){
    tud_int_handler(0);
}