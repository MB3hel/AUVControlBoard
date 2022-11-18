
#include <framework.h>
#include <FreeRTOS.h>
#include <task.h>
#include <led.h>
#include <usb.h>

/**
 * Thread to periodically blink the LED
 */
void led_thread(void *argument){
    while(1){
        led_toggle();
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

/**
 * Thread to periodically print a message via USB
 */
void usb_thread(void *argument){
    while(1){
        usb_write("Hello from control board!\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * Background service to process TinyUSB events
 */
void usb_service(void *argument){
    while(1){
        usb_process();
    }
}

int main(void){
    init_frameworks();
    led_init();
    usb_init();
    xTaskCreate(usb_service, "usb_service", 128, NULL, 1, NULL);
    xTaskCreate(usb_thread, "usb_thread", 128, NULL, 2, NULL);
    xTaskCreate(led_thread, "led_thread", 128, NULL, 2, NULL);
	vTaskStartScheduler();
}
