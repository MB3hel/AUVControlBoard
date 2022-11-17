
#include <framework.h>
#include <FreeRTOS.h>
#include <task.h>
#include <led.h>

void app_main(void *argument){
    led_init();
    while(1){
        led_toggle();
        vTaskDelay(250);
    }
}

int main(void){
    init_frameworks();
    xTaskCreate(app_main, "app_main", 128, NULL, tskIDLE_PRIORITY + 1, NULL);
	vTaskStartScheduler();
}
