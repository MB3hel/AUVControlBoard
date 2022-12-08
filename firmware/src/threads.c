#include <threads.h>
#include <tusb.h>
#include <limits.h>
#include <pccomm.h>
#include <cmdctrl.h>


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
                // Give the message to cmdctrl to handle (exclude last two bytes being CRC)
                // Use the message's CRC as it's identifier
                cmdctrl_handle_cmd(pccomm_read_crc, pccomm_read_buf, pccomm_read_len - 2);
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
            xTaskNotify(task_communicate_handle, NOTIF_COMM_DATA, eSetBits);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Thread management
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void threads_init(void){
    // Create RTOS threads
    // TODO
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
