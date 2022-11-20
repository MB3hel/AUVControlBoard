#include <led.h>
#include <framework.h>

void led_init(void){
    led_off();
}

void led_on(void){
#if defined(CONTROL_BOARD_V1)
    RED_LED_Set();
#elif defined(CONTROL_BOARD_V2)
    HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_SET);
#endif
}

void led_off(void){
#if defined(CONTROL_BOARD_V1)
    RED_LED_Clear();
#elif defined(CONTROL_BOARD_V2)
    HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
#endif
}

void led_toggle(void){
#if defined(CONTROL_BOARD_V1)
    RED_LED_Toggle();
#elif defined(CONTROL_BOARD_V2)
    HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
#endif
}
