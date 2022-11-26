
#include <framework.h>
#include <FreeRTOS.h>
#include <task.h>
#include <led.h>
#include <usb.h>
#include <delay.h>
#include <math.h>

/**
 * Thread to periodically blink the LED
 */
void led_thread(void *argument){
    while(1){
        led_toggle();
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

static void hsv_to_rgb(float *rgb, float h, float s, float v){
    // https://en.wikipedia.org/wiki/HSL_and_HSV#HSV_to_RGB
    // Note: Very inefficient on MCU. Could be converted to integer math and use if's instead of mod
    //       However, just a demo, so...
    float c = v * s;
    float x = c * (1 - fabsf(fmodf(h / 60.0f, 2) - 1));
    if(0.0f <= h && h < 60.0f){
        rgb[0] = c;
        rgb[1] = x;
        rgb[2] = 0;
    }else if(60.0f <= h && h < 120.0f){
        rgb[0] = x;
        rgb[1] = c;
        rgb[2] = 0;
    }else if(120.0f <= h && h < 180.0f){
        rgb[0] = 0;
        rgb[1] = c;
        rgb[2] = x;
    }else if(180.0f <= h && h < 240.0f){
        rgb[0] = 0;
        rgb[1] = x;
        rgb[2] = c;
    }else if(240.0f <= h && h < 300.0f){
        rgb[0] = x;
        rgb[1] = 0;
        rgb[2] = c;
    }else if(300.0f <= h && h < 360.0f){
        rgb[0] = c;
        rgb[1] = 0;
        rgb[2] = x;
    }else{
        rgb[0] = 0;
        rgb[1] = 0;
        rgb[2] = 0;
    }
    float m = v - c;
    rgb[0] = rgb[0] + m;
    rgb[1] = rgb[1] + m;
    rgb[2] = rgb[2] + m;

    // Normalize brightness (sum of all components is val)
    float sum = rgb[0] + rgb[1] + rgb[2];
    rgb[0] /= (sum / v);
    rgb[1] /= (sum / v);
    rgb[2] /= (sum / v);
}

/**
 * Thread to fade RGB led through colors
 */
void rgb_thread(void *argument){
    float hue = 0.0f;
    const float sat = 1.0f;
    const float val = 0.1f;
    float rgb[3];

    while(1){
        hue = fmodf(hue + 5.0f, 359.0f);
        hsv_to_rgb(rgb, hue, sat, val);
        led_rgb_set((uint8_t)(rgb[0] * 255), (uint8_t)(rgb[1] * 255), (uint8_t)(rgb[2] * 255));
        vTaskDelay(75 / portTICK_PERIOD_MS);
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
    delay_init();
    led_init();
    usb_init();
    led_off();
    xTaskCreate(usb_service, "usb_service", 128, NULL, 1, NULL);
    xTaskCreate(usb_thread, "usb_thread", 128, NULL, 2, NULL);
    xTaskCreate(led_thread, "led_thread", 128, NULL, 2, NULL);
    xTaskCreate(rgb_thread, "rgb_thread", 128, NULL, 2, NULL);
    vTaskStartScheduler();
}
