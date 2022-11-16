
#include <framework.h>

int main(void){
    init_frameworks();
    while(1){
        HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_SET);
        HAL_Delay(250);
        HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
        HAL_Delay(250);
    }
}
