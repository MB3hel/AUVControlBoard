#include <threads.h>
#include <tusb.h>
#include <limits.h>
#include <pccomm.h>
#include <cmdctrl.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Task configuration
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Stack sizes
#define TASK_USB_DEVICE_SSIZE               (192)                               // This is size used in CDC-MSC example
#define TASK_COMMUNICATE_SSIZE              (configMINIMAL_STACK_SIZE)

// Task priorities
#define TASK_USB_DEVICE_PRIORITY            (configMAX_PRIORITIES - 1)          // Must happen quickly for TUSB to work
#define TASK_COMMUNICATE_PRIORITY           (configMAX_PRIORITIES - 2)          // Must handle usb data quickly
                                                                                // Handling commands from PC is critical
// TODO: Task to handle sensor data will be lower priority

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///  Task Handles
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TaskHandle_t task_usb_device_handle;
TaskHandle_t task_communicate_handle;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Thread (task) functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define NOTIF_COMM_DATA             0x1


/**
 * Thread to handle messages from the pc
 */
void communicate_task(void *arg){
    uint32_t notification;

    while(1){
        // Wait until a notification is received (blocks this thread)
        // notification value is a set of 32 notification bits
        xTaskNotifyWait(pdFALSE, UINT32_MAX, &notification, portMAX_DELAY);

        // Handle the notification
        if(notification & NOTIF_COMM_DATA){
            // There is data to handle from the PC
            // Read and parse the data
            if(pccomm_read_and_parse()){
                // Inform cmdctrl there is a complete message
                cmdctrl_handle_message();
            }

            // If there is still data to handle, ensure flag is set again
            // This is necessary because read_and_parse may return true part way through reading data
            // Which means that usb_device_task will not notify again, but there is unhandled data
            if(tud_cdc_available())
                xTaskNotify(task_communicate_handle, NOTIF_COMM_DATA, eSetBits);
        }
    }
}

/**
 * Thread to handle TinyUSB device mode events
 */
void usb_device_task(void *argument){
    tud_init(BOARD_TUD_RHPORT);
    while(1){
        // This call will block thread until there is / are event(s)
        tud_task();

        // If data now available, notify the communication task
        if(tud_cdc_available()){
            // xTaskNotify(task_communicate_handle, NOTIF_COMM_DATA, eSetBits);
            asm("nop");
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Thread management
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void threads_init(void){
    // Create RTOS threads
    xTaskCreate(
        usb_device_task,
        "usb_device_task",
        TASK_USB_DEVICE_SSIZE,
        NULL,
        TASK_USB_DEVICE_PRIORITY,
        &task_usb_device_handle
    );
    xTaskCreate(
        communicate_task,
        "communicate_task",
        TASK_COMMUNICATE_SSIZE,
        NULL,
        TASK_COMMUNICATE_PRIORITY,
        &task_communicate_handle
    );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
