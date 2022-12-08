
#include <framework.h>
#include <FreeRTOS.h>
#include <task.h>
#include <led.h>
#include <delay.h>
#include <math.h>
#include <usb.h>
#include <thruster.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Task (thread) configuration
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Stack sizes
#define TASK_RGB_SSIZE                      (configMINIMAL_STACK_SIZE)
#define TASK_PCCOMM_SSIZE                   (configMINIMAL_STACK_SIZE)
#define TASK_USB_DEVICE_SSIZE               (192)                               // This is size used in CDC-MSC example

// Task priorities
#define TASK_RGB_PRIORITY                   (1)
#define TASK_PCCOMM_PRIORITY                (1)
#define TASK_USB_DEVICE_PRIORITY            (2)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Task handles
static TaskHandle_t task_rgb_handle;
static TaskHandle_t task_pccomm_handle;
static TaskHandle_t task_usb_device_handle;


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
void rgb_task(void *argument){
    float hue = 0.0f;
    const float sat = 1.0f;
    const float val = 0.1f;
    float rgb[3];

    while(1){
        hue = fmodf(hue + 5.0f, 359.0f);
        hsv_to_rgb(rgb, hue, sat, val);
        led_set((uint8_t)(rgb[0] * 255), (uint8_t)(rgb[1] * 255), (uint8_t)(rgb[2] * 255));
        vTaskDelay(75 / portTICK_PERIOD_MS);
    }
}

/**
 * Thread to periodically send message to PC via USB
 */
void pccomm_task(void *argument){
    while(1){
        const char msg[] = "Hello From ControlBoard!!!\r\n";
        usb_writestr(msg);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

int main(void){
    // -------------------------------------------------------------------------
    // System & Peripheral Initialization
    // -------------------------------------------------------------------------
    init_frameworks();
    delay_init();
    led_init();
    usb_init();
    thruster_init();
    // -------------------------------------------------------------------------
    
    // -------------------------------------------------------------------------
    // RTOS task creation
    // -------------------------------------------------------------------------
    xTaskCreate(
        usb_device_task, 
        "usb_device_task", 
        TASK_USB_DEVICE_SSIZE, 
        NULL, 
        TASK_USB_DEVICE_PRIORITY, 
        &task_usb_device_handle
    );
    xTaskCreate(
        pccomm_task,
        "pccomm_task", 
        TASK_PCCOMM_SSIZE, 
        NULL, 
        TASK_PCCOMM_PRIORITY, 
        &task_pccomm_handle
    );
    xTaskCreate(
        rgb_task, 
        "rgb_task", 
        TASK_RGB_SSIZE, 
        NULL, 
        TASK_RGB_PRIORITY, 
        &task_rgb_handle
    );
    // -------------------------------------------------------------------------

    // -------------------------------------------------------------------------
    // RTOS Startup
    // -------------------------------------------------------------------------
    vTaskStartScheduler();
    // -------------------------------------------------------------------------

    // Start scheduler should never return. This should never run, but is
    // included to make debugging easier in case it does
    while(1){
        asm("nop");
    }
}
