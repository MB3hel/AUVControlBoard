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

#pragma once

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Notification values to tasks
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CMDCTRL Task
#define NOTIF_CMDCTRL_PCDATA                0x1     // Notify CMDCTRL that there is data from PC
#define NOTIF_FEED_WDT                      0x2     // Notify CMDCTRL thread to feed WDT
#define NOTIF_SIM_STAT                      0x4     // Notify CMDCTRL thread to send SIMSTAT message (if sim hijacked)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Task configuration
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Stack sizes
#define TASK_USB_DEVICE_SSIZE               (192)                               // This is size used in CDC-MSC example
#define TASK_CMDCTRL_SSIZE                  (6*configMINIMAL_STACK_SIZE)        // May call many levels of functions
#define TASK_IMU_SSIZE                      (6*configMINIMAL_STACK_SIZE)        // May call many levels of functions
#define TASK_DEPTH_SSZIE                    (6*configMINIMAL_STACK_SIZE)        // May call many levels of functions

// Task priorities
#define TASK_USB_DEVICE_PRIORITY            (configMAX_PRIORITIES - 1)          // Must happen quickly for TUSB to work
#define TASK_CMDCTRL_PRIORITY               (configMAX_PRIORITIES - 2)          // Must handle usb data quickly
                                                                                // Handling commands from PC is critical
#define TASK_IMU_PRIORITY                   (configMAX_PRIORITIES - 3)          // Sensor data acquisition less critical
#define TASK_DEPTH_PRIORITY                 (configMAX_PRIORITIES - 3)          // Sensor data acquisition less critical

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///  RTOS object handles
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Task handles
extern TaskHandle_t usb_device_task;
extern TaskHandle_t cmdctrl_task;
extern TaskHandle_t imu_task;
extern TaskHandle_t depth_task;

// Timers
extern TimerHandle_t wdt_feed_timer;
extern TimerHandle_t sim_timer;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///  Initialization & management functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void app_init(void);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
