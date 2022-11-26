
#include <delay.h>
#include <framework.h>


void delay_init(void){
    // Blocking delays implemented using DWT
    // Note: disable then enable seems to be required on STM32
    //       Since it won't hurt elsewhere, just do it everywhere
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;        // Disable TCR
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;         // Enable TCR
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;                   // Disable clock cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;                    // Enable clock cycle counter
    DWT->CYCCNT = 0;                                        // Reset counter on enable
}

void delay_us(unsigned int us){
    uint32_t start = DWT->CYCCNT;
    us *= (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < us);
}

void delay_ms(unsigned int ms){
    uint32_t start = DWT->CYCCNT;
    ms *= (SystemCoreClock / 1000);
    while ((DWT->CYCCNT - start) < ms);
}
