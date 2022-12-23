/*
 * Copyright 2022 Marcus Behel
 * 
 * This file is part of AUVControlBoard-Firmware.
 * 
 * AUVControlBoard-Firmware is free software: you can redistribute it and/or modify it under the terms of the GNU 
 * General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 * 
 * AUVControlBoard-Firmware is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even 
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for 
 * more details.
 * 
 * You should have received a copy of the GNU General Public License along with AUVControlBoard-Firmware. If not, see 
 * <https://www.gnu.org/licenses/>. 
 * 
 */

#include <app.h>
#include <tusb.h>
#include <limits.h>
#include <pccomm.h>
#include <cmdctrl.h>
#include <bno055.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///  RTOS object handles
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Task handles
TaskHandle_t usb_device_task;
TaskHandle_t cmdctrl_task;
TaskHandle_t imu_task;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Thread (task) functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Command and control task
 * cmdctrl (and motor_control) functions are not thread safe, thus only this thread
 * should ever call such functions
 */
void cmdctrl_task_func(void *arg){
    uint32_t notification;
    
    while(1){
        // Wait until a notification is received (blocks this thread)
        // notification value is a set of 32 notification bits
        xTaskNotifyWait(pdFALSE, UINT32_MAX, &notification, portMAX_DELAY);

        // Handle the notification
        if(notification & NOTIF_CMDCTRL_PCDATA){
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
                xTaskNotify(cmdctrl_task, NOTIF_CMDCTRL_PCDATA, eSetBits);
        }
    }
}

/**
 * Thread to handle TinyUSB device mode events
 */
void usb_device_task_func(void *argument){
    tud_init(BOARD_TUD_RHPORT);
    while(1){
        // This call will block thread until there is / are event(s)
        tud_task();

        // If data now available, notify the communication task
        if(tud_cdc_available()){
            xTaskNotify(cmdctrl_task, NOTIF_CMDCTRL_PCDATA, eSetBits);
        }
    }
}

/**
 * Thread to handle IMU data
 * I2C functions are thread safe (by mutex), so thread will block
 * until it has I2C
 */
void imu_task_func(void *argument){
    // Tracks if IMU configured currently
    bool configured = false;
    unsigned int read_failures = 0;
    bno055_data data;

    bno055_init();
    while(1){
        if(!configured){
            // Configure IMU. Will succeed if IMU connected.
            configured = bno055_configure();

            // If configure fails, wait 1 second before trying again
            if(!configured)
                vTaskDelay(pdMS_TO_TICKS(1000));
            else
                cmdctrl_bno055_status(true);
        }else{
            // IMU is connected and has been configured
            // Periodically read data
            if(bno055_read(&data)){
                read_failures = 0;
                cmdctrl_bno055_data(data);
            }else{
                // Too many failures. Assume IMU no longer connected (or has reset)
                read_failures++;
                if(read_failures > 5){
                    configured = false;
                    cmdctrl_bno055_status(false);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(15));
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// App initialization & management
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void app_init(void){
    // Create RTOS threads
    xTaskCreate(
        usb_device_task_func,
        "usbd_task",
        TASK_USB_DEVICE_SSIZE,
        NULL,
        TASK_USB_DEVICE_PRIORITY,
        &usb_device_task
    );
    xTaskCreate(
        cmdctrl_task_func,
        "cmdctrl_task",
        TASK_CMDCTRL_SSIZE,
        NULL,
        TASK_CMDCTRL_PRIORITY,
        &cmdctrl_task
    );
    xTaskCreate(
        imu_task_func,
        "imu_task",
        TASK_IMU_SSIZE,
        NULL,
        TASK_IMU_PRIORITY,
        &imu_task
    );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
