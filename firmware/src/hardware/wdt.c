/*
 * Copyright 2023 Marcus Behel
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

#include <hardware/wdt.h>
#include <framework.h>


#ifdef CONTROL_BOARD_V1

void wdt_init(void){
    WDT_TimeoutPeriodSet(0x8);          // Configure to timeout after 2048 cycles (1024 Hz clock) ~ 2sec
    WDT_DisableWindowMode();
    WDT_Enable();
}

void wdt_feed(void){
    WDT_Clear();
}

#endif // CONTROL_BOARD_V1


#ifdef CONTROL_BOARD_V2

extern IWDG_HandleTypeDef hiwdg;

extern void MX_IWDG_Init(void);

void wdt_init(void){
    // Init handled in CubeMX generated code
    // LSI = 32kHz on this board
    // Prescaler = 32  ->  1024 counts / second
    // Reload configured to 2048 ~= 2 second timeout
    MX_IWDG_Init();
    
    // Freeze watchdog when core halted
    __HAL_DBGMCU_FREEZE_IWDG();
}

void wdt_feed(void){
    HAL_IWDG_Refresh(&hiwdg);
}

#endif // CONTROL_BOARD_V2


#if defined(CONTROL_BOARD_SIM_LINUX) || defined(CONTROL_BOARD_SIM_MAC)

#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <debug.h>

// Simulated WDT. Allows detection of deadlocks that would trigger WDT in hardware using SimCB
// Implemented by comparing current time to last feed time periodically

#define WDT_TIMEOUT     2000        // ms
#define WDT_PRECISION   250         // ms


static unsigned long long feed_time;
static pthread_t wdt_tid;

unsigned long long millis(){
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return (t.tv_nsec / 1000000) + (t.tv_sec * 1000);
}

static void *wdt_thread(void *arg){
    // Assume last fed when this thread starts so timeout occurs WDT_TIMEOUT
    // after thread is running at the earliest
    feed_time = millis();
    while(1){
        struct timespec t;
        t.tv_nsec = (WDT_PRECISION % 1000) * 1000000;
        t.tv_sec = WDT_PRECISION / 1000;
        nanosleep(&t, NULL);
        if(millis() - feed_time > WDT_TIMEOUT){
            // Watchdog timeout occurred!!!
            debug_halt(HALT_EC_WDOG);
        }
    }
    return NULL;
}

void wdt_init(void){
    // Mask all signals on creating thread
    // Created thread will inherit signal mask
    // And non-RTOS threads must mask all signals
    // This is a portable way to ensure all signals are masked
    // upon creation of the child thread
    sigset_t fullset, origset;
    sigfillset(&fullset);
    pthread_sigmask(SIG_SETMASK, &fullset, &origset);
    pthread_create(&wdt_tid, NULL, wdt_thread, NULL);
    pthread_sigmask(SIG_SETMASK, &origset, NULL);
}

void wdt_feed(void){
    feed_time = millis();
}

#endif // CONTROL_BOARD_SIM_LINUX || CONTROL_BOARD_SIM_MACOS

#ifdef CONTROL_BOARD_SIM_WIN

#include <windows.h>
#include <sysinfoapi.h>
#include <synchapi.h>
#include <debug.h>

// Simulated WDT. Allows detection of deadlocks that would trigger WDT in hardware using SimCB
// Implemented by comparing current time to last feed time periodically

#define WDT_TIMEOUT     2000        // ms
#define WDT_PRECISION   250         // ms


static unsigned long long feed_time;
static HANDLE wdt_thread_handle;

unsigned long long millis(){
    return GetTickCount64();
}

static DWORD WINAPI wdt_thread(void *arg){
    // Assume last fed when this thread starts so timeout occurs WDT_TIMEOUT
    // after thread is running at the earliest
    feed_time = millis();
    while(1){
        Sleep(WDT_PRECISION);
        if(millis() - feed_time > WDT_TIMEOUT){
            // Watchdog timeout occurred!!!
            debug_halt(HALT_EC_WDOG);
        }
    }
    return 0;
}

void wdt_init(void){
    wdt_thread_handle = CreateThread(NULL, 0, wdt_thread, NULL, 0, NULL);
}

void wdt_feed(void){
    feed_time = millis();
}

#endif // CONTROL_BOARD_SIM_WIN