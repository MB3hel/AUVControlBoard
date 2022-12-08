#include <led.h>
#include <framework.h>


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

void led_init(void){
    // Configured in generated code to a PWM frequency of ~500Hz (96MHz / 3 / 65535 = 488MHz)
    // 16-bit resolution PWM (0-65535)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);           // R
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);           // G
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);           // B

    led_set(0, 0, 0);
}


void led_set(uint8_t r, uint8_t g, uint8_t b){
    // 255 - [r,g,b] because LED is common anode
    // Thus, lowest duty cycle is max brightness
    // Multiply by 255 so 0 -> 0 and 255 -> 65535
    // This timer is 16-bit but colors specified as 8-bit

    TIM1->CCR1 = (255 - r) * 257;
    TIM4->CCR1 = (255 - g) * 257;
    TIM4->CCR2 = (255 - b) * 257;
}
