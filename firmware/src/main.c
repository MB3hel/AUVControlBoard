
#include <framework.h>

#define DELAY_TIME 250

int main(void){
    init_frameworks();
    while(1){
#if defined(CONTROL_BOARD_V1)
        gpio_set_pin_level(RED_LED, true);
        delay_ms(DELAY_TIME);
        gpio_set_pin_level(RED_LED, false);
        delay_ms(DELAY_TIME);
#elif defined(CONTROL_BOARD_V2)
        HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_SET);
        HAL_Delay(DELAY_TIME);
        HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
        HAL_Delay(DELAY_TIME);
#endif
    }
}
