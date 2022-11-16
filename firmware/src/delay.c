#include <delay.h>
#include <framework.h>

void delay_initialize(void){
#if defined(CONTROL_BOARD_V1)
    // ASF us driver is wrong (too fast), so implement a custom one using DWT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;         // Enable TCR
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;                    // Enable clock cycle counter
    DWT->CYCCNT = 0;                                        // Reset counter on enable
#elif defined(CONTROL_BOARD_V2)
    // HAL has no delay driver for microseconds, so build one using DWT
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;        // Disable TCR
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;         // Enable TCR
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;                   // Disable clock cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;                    // Enable clock cycle counter
    DWT->CYCCNT = 0;                                        // Reset counter on enable
#endif
}

void delay_micros(unsigned int us){
#if defined(CONTROL_BOARD_V1)
    uint32_t start = DWT->CYCCNT;
    us *= (CONF_CPU_FREQUENCY / 1000000);
    while ((DWT->CYCCNT - start) < us);
#elif defined(CONTROL_BOARD_V2)
    uint32_t start = DWT->CYCCNT;
    us *= (HAL_RCC_GetHCLKFreq() / 1000000);
    while ((DWT->CYCCNT - start) < us);
#endif
}

void delay_millis(unsigned int ms){
#if defined(CONTROL_BOARD_V1)
    delay_ms(ms);
#elif defined(CONTROL_BOARD_V2)
    HAL_Delay(ms);
#endif
}

void delay_seconds(unsigned int sec){
    for(; sec > 0; --sec){
        delay_millis(1000);
    }
}
