/**
 * @file usb.c
 * @author Marcus Behel
 */

#include <usb.h>
#include <sam.h>
#include <clocks.h>
#include <tusb.h>
#include <timers.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void usb_init(void){
    // Note: USB requires 48MHz clock
    GCLK->PCHCTRL[USB_GCLK_ID].bit.GEN = CLOCKS_GCLK_48M;           // Select 48MHz clock for USB
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

	tud_init(BOARD_TUD_RHPORT);                                     // Initialize TinyUSB
}

void usb_process(void){
    tud_task();                                                     // TinyUSB task for device 
    
    // Handle data from CDC interface (echo)
    if (tud_cdc_connected()){
        if(tud_cdc_available()){
            char buf[64];
            uint32_t count = tud_cdc_read(buf, sizeof(buf));
            tud_cdc_write(buf, count);
            tud_cdc_write_flush();
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// TinyUSB Callbacks
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts){
    // Run when line state changes
    // DTR = Data Terminal Ready
    // RTS = Ready to Send
    // DRT usually set when terminal connected

    // sam-ba upload protocol uses a 1200bps "touch" to trigger a reset
    // Handle this as expected
    // Not strictly necessary, but prevents having to press reset button to program
    if (!dtr && itf == 0) {
        cdc_line_coding_t coding;
        tud_cdc_get_line_coding(&coding);
        if (coding.bit_rate == 1200){
            // Special things to reboot to bootloader instead of main program
            // Must match bootloader. Taken from Adafruit/ArduinoCore-samd Reset.cpp
            // THIS IS SPECIFIC TO ITSY BITSY M4!!!
            #define DOUBLE_TAP_MAGIC 			0xf01669efUL
	        #define BOOT_DOUBLE_TAP_ADDRESS     (HSRAM_ADDR + HSRAM_SIZE - 4)
            unsigned long *a = (unsigned long *)BOOT_DOUBLE_TAP_ADDRESS;
	        *a = DOUBLE_TAP_MAGIC;

            // Reset the system now
            TIMERS_WDT_RESET_NOW();
        }
    }
}

void tud_cdc_rx_cb(uint8_t itf){
    // Run when CDC receives data from host
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// IRQ Handlers
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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