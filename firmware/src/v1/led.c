#include <led.h>
#include <framework.h>
#include <delay.h>
#include <FreeRTOS.h>
#include <task.h>


void led_init(void){
    led_off();
    led_rgb_set(0, 0, 0);
}

void led_on(void){
    // No longer implemented. Pin used for THR1
    asm("nop");
}

void led_off(void){
    // No longer implemented. Pin used for THR1
    asm("nop");
}

void led_toggle(void){
    // No longer implemented. Pin used for THR1
    asm("nop");
}

static inline void dotstar_write(uint8_t val){
    for (uint8_t bit = 8; bit > 0; bit--){
        if((val & (1 << (bit - 1))))
            DS_DAT_Set();
        else
            DS_DAT_Clear();
        DS_CLK_Set();
        delay_us(1);
        DS_CLK_Clear();
        delay_us(1);
    }
}

void led_rgb_set(uint8_t r, uint8_t g, uint8_t b){
    // ItsyBitsy M4 includes an RGB "dotstar" LED
    // The pins used for this are setup as gpio outputs in the MCC project
    // NOTE: Since this is bitbanged and requires fairly precise us level delays,
    //       this must disallow context switches while running
    taskENTER_CRITICAL();
    dotstar_write(0x00);
    dotstar_write(0x00);
    dotstar_write(0x00);
    dotstar_write(0x00);
    dotstar_write(0xFF);
    dotstar_write(b);
    dotstar_write(g);
    dotstar_write(r);
    dotstar_write(0xFF);
    taskEXIT_CRITICAL();
}
