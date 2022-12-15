#pragma once

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Notification values to tasks
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CMDCTRL Task
#define NOTIF_CMDCTRL_PCDAATA               0x1                 // Notify CMDCTRL that there is data from PC


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Task configuration
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Stack sizes
#define TASK_USB_DEVICE_SSIZE               (192)                               // This is size used in CDC-MSC example
#define TASK_CMDCTRL_SSIZE                  (6*configMINIMAL_STACK_SIZE)        // May call many levels of functions
#define TASK_IMU_SSIZE                      (3*configMINIMAL_STACK_SIZE)        // Not too many levels of fcn calls

// Task priorities
#define TASK_USB_DEVICE_PRIORITY            (configMAX_PRIORITIES - 1)          // Must happen quickly for TUSB to work
#define TASK_CMDCTRL_PRIORITY               (configMAX_PRIORITIES - 2)          // Must handle usb data quickly
                                                                                // Handling commands from PC is critical
#define TASK_IMU_PRIORITY                   (configMAX_PRIORITIES - 3)          // Sensor data acquisition less critical

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///  RTOS object handles
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Task handles
extern TaskHandle_t usb_device_task;
extern TaskHandle_t cmdctrl_task;
extern TaskHandle_t imu_task;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///  Initialization & management functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void app_init(void);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
