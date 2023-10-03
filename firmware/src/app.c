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
#include <limits.h>
#include <pccomm.h>
#include <cmdctrl.h>
#include <bno055.h>
#include <ms5837.h>
#include <hardware/wdt.h>


#if defined(CONTROL_BOARD_V1) || defined(CONTROL_BOARD_V2)
#include <tusb.h>
#endif


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///  RTOS object handles
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Task handles
TaskHandle_t usb_device_task;
TaskHandle_t cmdctrl_task;
TaskHandle_t imu_task;
TaskHandle_t depth_task;

// Timers
TimerHandle_t wdt_feed_timer;
TimerHandle_t sim_timer;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Timer handlers
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void wdt_feed_timer_handler(TimerHandle_t handle){
    (void)handle;

    xTaskNotify(cmdctrl_task, NOTIF_FEED_WDT, eSetBits);
}

void sim_timer_handler(TimerHandle_t handle){
    (void)handle;

    if(cmdctrl_sim_hijacked)
        xTaskNotify(cmdctrl_task, NOTIF_SIM_STAT, eSetBits);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Thread (task) functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(CONTROL_BOARD_V1) || defined(CONTROL_BOARD_V2)
#define has_usb_data() tud_cdc_available()
#else
// SimCB uses STDIO instead of TinyUSB
// TODO
#define has_usb_data() false
#endif

/**
 * Command and control task
 * cmdctrl (and motor_control) functions are not thread safe, thus only this thread
 * should ever call such functions
 */
void cmdctrl_task_func(void *arg){
    (void)arg;

    uint32_t notification;

    xTimerStart(wdt_feed_timer, portMAX_DELAY);
    xTimerStart(sim_timer, portMAX_DELAY);


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
            if(has_usb_data())
                xTaskNotify(cmdctrl_task, NOTIF_CMDCTRL_PCDATA, eSetBits);
        }
        if(notification & NOTIF_SIM_STAT){
            cmdctrl_send_simstat();
        }
        if(notification & NOTIF_FEED_WDT){
            wdt_feed();
        }
        if(notification & NOTIF_NO_HIJACK){
            cmdctrl_simhijack(false);
        }
    }
}

/**
 * Thread to handle TinyUSB device mode events
 */
void usb_device_task_func(void *argument){
    (void)argument;

#if defined(CONTROL_BOARD_V1) || defined(CONTROL_BOARD_V2)
    tud_init(BOARD_TUD_RHPORT);
    while(1){
        // This call will block thread until there is / are event(s)
        tud_task();

        // If data now available, notify the communication task
        if(tud_cdc_available()){
            xTaskNotify(cmdctrl_task, NOTIF_CMDCTRL_PCDATA, eSetBits);
        }
    }
#endif
}

/**
 * Thread to handle IMU data
 * I2C functions are thread safe (by mutex), so thread will block
 * until it has I2C
 */
void imu_task_func(void *argument){
    (void)argument;

    // Tracks if IMU configured currently
    bool configured = false;
    unsigned int read_failures = 0;
    bno055_data data;

    bno055_init();
    while(1){
        if(!configured && !cmdctrl_sim_hijacked){
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

/**
 * Thread to handle depth sensor data
 * I2C functions are thread safe (by mutex), so thread will block
 * until it has I2C
 */
void depth_task_func(void *argument){
    (void)argument;

    // Tracks if IMU configured currently
    bool configured = false;
    unsigned int read_failures = 0;
    ms5837_data data;

    ms5837_init();
    while(1){
        if(!configured && !cmdctrl_sim_hijacked){
            // Configure sensor. Will succeed if sensor connected.
            configured = ms5837_configure();

            // If configure fails, wait 1 second before trying again
            if(!configured)
                vTaskDelay(pdMS_TO_TICKS(1000));
            else
                cmdctrl_ms5837_status(true);
        }else{
            // sensor is connected and has been configured
            // Periodically read data
            if(ms5837_read(&data)){
                read_failures = 0;
                cmdctrl_ms5837_data(data);
            }else{
                // Too many failures. Assume sensor no longer connected (or has reset)
                read_failures++;
                if(read_failures > 5){
                    configured = false;
                   cmdctrl_ms5837_status(false);
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
    // Create RTOS objects
    wdt_feed_timer = xTimerCreate("wdt_feed_timer", pdMS_TO_TICKS(350), pdTRUE, NULL, wdt_feed_timer_handler);
    sim_timer = xTimerCreate("sim_timer", pdMS_TO_TICKS(20), pdTRUE, NULL, sim_timer_handler);


    // Create RTOS threads
#if defined(CONTROL_BOARD_V1) || defined(CONTROL_BOARD_V2)
    xTaskCreate(
        usb_device_task_func,
        "usbd_task",
        TASK_USB_DEVICE_SSIZE,
        NULL,
        TASK_USB_DEVICE_PRIORITY,
        &usb_device_task
    );
#endif
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
    xTaskCreate(
        depth_task_func,
        "depth_task",
        TASK_DEPTH_SSZIE,
        NULL,
        TASK_DEPTH_PRIORITY,
        &depth_task
    );
}

void app_handle_usb_disconnect(void){
    // When USB disconnects, revert to normal operation
    if(cmdctrl_sim_hijacked)
        xTaskNotify(cmdctrl_task, NOTIF_NO_HIJACK, eSetBits);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
