#include <led.h>
#include <framework.h>

void led_init(void){
    led_off();
}

void led_on(void){
#if defined(CONTROL_BOARD_V1)
    gpio_set_pin_level(RED_LED, true);
#elif defined(CONTROL_BOARD_V2)
    HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_SET);
#endif
}

void led_off(void){
#if defined(CONTROL_BOARD_V1)
    gpio_set_pin_level(RED_LED, false);
#elif defined(CONTROL_BOARD_V2)
    HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
#endif
}

void led_toggle(void){
#if defined(CONTROL_BOARD_V1)
    gpio_toggle_pin_level(RED_LED);
#elif defined(CONTROL_BOARD_V2)
    HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
#endif
}
