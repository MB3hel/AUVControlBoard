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
#include <imu.h>
#include <depth.h>
#include <hardware/wdt.h>
#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <task.h>
#include <debug.h>
#include <hardware/usb.h>

// TODO: Remove
#include <stdio.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Constants / Configuration parameters
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Task Notifications to CMDCTRL task
#define NOTIF_PCDATA                        0x1     // Notify that there is data from PC
#define NOTIF_FEED_WDT                      0x2     // Notify to feed WDT
#define NOTIF_SIM_STAT                      0x4     // Notify to send SIMSTAT message (if sim hijacked)
#define NOTIF_UART_CLOSE                    0x8     // Notify thread that UART is closed

// Stack sizes
#define TASK_USB_SSIZE                      192
#define TASK_CMDCTRL_SSIZE                  768
#define TASK_IMU_SSIZE                      768
#define TASK_DEPTH_SSZIE                    768

// Task priorities
#define TASK_USB_PRIORITY                   (configMAX_PRIORITIES - 1)      // Must happen quickly for TUSB to work
#define TASK_CMDCTRL_PRIORITY               (configMAX_PRIORITIES - 2)      // Comms more important than sensor data
#define TASK_IMU_PRIORITY                   (configMAX_PRIORITIES - 3)      // Sensor data acquisition less critical
#define TASK_DEPTH_PRIORITY                 (configMAX_PRIORITIES - 3)      // Sensor data acquisition less critical

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///  RTOS object handles
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Task handles
static TaskHandle_t usb_task;
static TaskHandle_t cmdctrl_task;
static TaskHandle_t imu_task;
static TaskHandle_t depth_task;

// Timers
static TimerHandle_t wdt_feed_timer;
static TimerHandle_t sim_timer;
static TimerHandle_t tcp_timer;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Thread (task) functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Thread to handle USB events (must happen quickly for TUSB to work properly)
 */
static void usb_task_func(void *argument){
    (void)argument;

    usb_init();
#ifdef CONTROL_BOARD_SIM
    xTimerStart(tcp_timer, portMAX_DELAY);
#endif
    while(1){
        // This call will block thread until there is / are event(s)
        // Blocks until there may be data
        usb_process();

        // If data now available, notify the communication task
        if(usb_avail()){
            xTaskNotify(cmdctrl_task, NOTIF_PCDATA, eSetBits);
        }
    }
}


/**
 * Thread handling communication and motor control logic
 */
static void cmdctrl_task_func(void *argument){
    (void)argument;

    uint32_t notification;

    xTimerStart(wdt_feed_timer, portMAX_DELAY);
    xTimerStart(sim_timer, portMAX_DELAY);


    while(1){
        // Wait until a notification is received (blocks this thread)
        // notification value is a set of 32 notification bits
        xTaskNotifyWait(pdFALSE, UINT32_MAX, &notification, portMAX_DELAY);

        // ---------------------------------------------------------------------
        // Handle any notifications (can be multiple at a time)
        // ---------------------------------------------------------------------
        if(notification & NOTIF_PCDATA){
            // There is data to handle from the PC
            // Read and parse the data. 
            // Handle any complete messages appropriately.
            while(1){
                if(pccomm_read_and_parse())
                    cmdctrl_handle_message();
                else
                    break; // Got to end of data without a complete message
            }
        }
        if(notification & NOTIF_SIM_STAT){
            // Timer indicates it is time to send simstat
            cmdctrl_send_simstat();
        }
        if(notification & NOTIF_FEED_WDT){
            // Timer indicates it is time to feed watchdog
            wdt_feed();
        }
        if(notification & NOTIF_UART_CLOSE){
            // UART connection closed. Revert out of simhijack
            cmdctrl_simhijack(false);
        }
        // ---------------------------------------------------------------------
    }
}

/**
 * Thread to handle IMU data
 * I2C functions are thread safe (by mutex), so thread will block
 * until it has I2C
 */
static void imu_task_func(void *argument){
    (void)argument;

    imu_init();
    while(1){
        if(imu_read()){
            // Wait 15ms between reads
            vTaskDelay(pdMS_TO_TICKS(15));
        }else if(imu_get_sensor() == IMU_NONE){
            // Read failed b/c no IMU connected. Delay longer before trying again.
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

/**
 * Thread to handle depth sensor data
 * I2C functions are thread safe (by mutex), so thread will block
 * until it has I2C
 */
static void depth_task_func(void *argument){
    (void)argument;

    depth_init();
    while(1){
        if(depth_read()){
            // Wait 15ms between reads
            vTaskDelay(pdMS_TO_TICKS(15));
        }else if(depth_get_sensor() == IMU_NONE){
            // Read failed b/c no IMU connected. Delay longer before trying again.
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Timer handlers
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void wdt_feed_timer_handler(TimerHandle_t handle){
    (void)handle;

    xTaskNotify(cmdctrl_task, NOTIF_FEED_WDT, eSetBits);
}

static void sim_timer_handler(TimerHandle_t handle){
    (void)handle;

    if(cmdctrl_sim_hijacked)
        xTaskNotify(cmdctrl_task, NOTIF_SIM_STAT, eSetBits);
}

#ifdef CONTROL_BOARD_SIM
// When built for SIM, USB is simulated using TCP
// Detecting TCP disconnects though is necessary to accept new connections
// Detecting disconnects requires a write
// Thus, make sure something is written every second
static void tcp_timer_handler(TimerHandle_t handle){
    // Write at pccomm layer b/c pccomm write is thread safe, but USB is not.
    uint8_t msg[] = {'H', 'E', 'A', 'R', 'T', 'B', 'E', 'A', 'T'};
    if(usb_initialized)
        pccomm_write(msg, sizeof(msg));
}
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// App initialization & management
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void app_init(void){
    // Create RTOS objects
    wdt_feed_timer = xTimerCreate("wdt_feed_timer", pdMS_TO_TICKS(350), pdTRUE, NULL, wdt_feed_timer_handler);
    sim_timer = xTimerCreate("sim_timer", pdMS_TO_TICKS(20), pdTRUE, NULL, sim_timer_handler);
#ifdef CONTROL_BOARD_SIM
    // See comments above tcp_timer_handler for purpose
    tcp_timer = xTimerCreate("tcp_timer", pdMS_TO_TICKS(1000), pdTRUE, NULL, tcp_timer_handler);
#endif

    // Create RTOS threads
    xTaskCreate(
        usb_task_func,
        "usbd_task",
        TASK_USB_SSIZE,
        NULL,
        TASK_USB_PRIORITY,
        &usb_task
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
    xTaskCreate(
        depth_task_func,
        "depth_task",
        TASK_DEPTH_SSZIE,
        NULL,
        TASK_DEPTH_PRIORITY,
        &depth_task
    );
}

void app_handle_uart_closed(void){
    if(cmdctrl_sim_hijacked)
        xTaskNotify(cmdctrl_task, NOTIF_UART_CLOSE, eSetBits);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// FreeRTOS support code
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// ---------------------------------------------------------------------------------------------------------------------
// General Hooks
// ---------------------------------------------------------------------------------------------------------------------

void vAssertCalled(const char *file, unsigned int line){
    (void)file;
    (void)line;

    debug_halt(HALT_EC_ASSERT);
}

void vApplicationMallocFailedHook(void){
    debug_halt(HALT_EC_MALLOC_FAIL);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName){
    (void)xTask;
    (void)pcTaskName;
    
    debug_halt(HALT_EC_SOVERFLOW);
}

#ifdef CONTROL_BOARD_SIM
void vApplicationTickHook(void){
    // Interrupts are simulated from this hook
    // Anything called from this hook must be treated as if it is running in an ISR
    usb_sim_interrupts();
}
#endif

// ---------------------------------------------------------------------------------------------------------------------



// ---------------------------------------------------------------------------------------------------------------------
// Static Allocation Support
// ---------------------------------------------------------------------------------------------------------------------

#if (configSUPPORT_STATIC_ALLOCATION == 1)

// Function implementations from https://www.freertos.org/a00110.html#configSUPPORT_STATIC_ALLOCATION

/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
/* If the buffers to be provided to the Idle task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*-----------------------------------------------------------*/

/* configSUPPORT_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize )
{
/* If the buffers to be provided to the Timer task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xTimerTaskTCB;
static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
    task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configTIMER_TASK_STACK_DEPTH is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

#endif

// ---------------------------------------------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
