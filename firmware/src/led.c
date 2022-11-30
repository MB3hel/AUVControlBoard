#include <led.h>
#include <framework.h>
#include <delay.h>
#include <FreeRTOS.h>
#include <task.h>


#if defined(CONTROL_BOARD_V2)
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
#endif

void led_init(void){
    led_off();
    led_rgb_set(0, 0, 0);

#if defined(CONTROL_BOARD_V2)
    // Configured in generated code to a PWM frequency of ~500Hz (96MHz / 3 / 65535 = 488MHz)
    // 16-bit resolution PWM (0-65535)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);           // R
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);           // G
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);           // B
#endif
}

void led_on(void){
#if defined(CONTROL_BOARD_V1)
    RED_LED_Set();
#elif defined(CONTROL_BOARD_V2)
    HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
#endif
}

void led_off(void){
#if defined(CONTROL_BOARD_V1)
    RED_LED_Clear();
#elif defined(CONTROL_BOARD_V2)
    HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_SET);
#endif
}

void led_toggle(void){
#if defined(CONTROL_BOARD_V1)
    RED_LED_Toggle();
#elif defined(CONTROL_BOARD_V2)
    HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
#endif
}

#if defined(CONTROL_BOARD_V1)
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
#endif

void led_rgb_set(uint8_t r, uint8_t g, uint8_t b){
#if defined(CONTROL_BOARD_V1)
    // ItsyBitsy M4 includes an RGB "dotstar" LED
    // The pins used for this are setup as gpio outputs in the MCC project
    // NOTE: Since this is bitbanged and requires fairly precise us level delays,
    //       this must disallow ocntext switches while running
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
#elif defined(CONTROL_BOARD_V2)
    // LED is common anode, thus red full on = min duty cycle pwm
    TIM1->CCR1 = (255 - r) * 257;
    TIM4->CCR1 = (255 - g) * 257;
    TIM4->CCR2 = (255 - b) * 257;
#endif
}
