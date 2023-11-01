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

#include <hardware/delay.h>
#include <framework.h>

#if defined(CONTROL_BOARD_V1) || defined(CONTROL_BOARD_V2)

void delay_init(void){
    // Blocking delays implemented using DWT
    // Note: disable then enable seems to be required on STM32
    //       Since it won't hurt elsewhere, just do it everywhere
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;        // Disable TCR
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;         // Enable TCR
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;                   // Disable clock cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;                    // Enable clock cycle counter
    DWT->CYCCNT = 0;                                        // Reset counter on enable
}

void delay_us(unsigned int us){
    uint32_t start = DWT->CYCCNT;
    us *= (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < us);
}

void delay_ms(unsigned int ms){
    uint32_t start = DWT->CYCCNT;
    ms *= (SystemCoreClock / 1000);
    while ((DWT->CYCCNT - start) < ms);
}

#endif // CONTROL_BOARD_V1 || CONTROL_BOARD_V2

#if defined(CONTROL_BOARD_SIM_LINUX) || defined(CONTROL_BOARD_SIM_MACOS)

#include <time.h>

// SimCB
void delay_init(void){}

void delay_us(unsigned int us){
    // ONLY use nanosleep (due to how threading works in Posix port of FreeRTOS)
    struct timespec t = {.tv_nsec = 0, .tv_sec = 0};
    struct timespec r = {.tv_nsec = 0, .tv_sec = 0};
    while(us >= 1000000){
        t.tv_sec++;
        us -= 1000000;
    }
    t.tv_nsec = us * 1000;

    while(1){
        if(nanosleep(&t, &r) == 0){
            // Success
            break;
        }else{
            // Interrupted. New time to sleep is remaining
            // Loop to continue sleep
            t = r;
        }
    }
}

void delay_ms(unsigned int ms){
    // ONLY use nanosleep (due to how threading works in Posix port of FreeRTOS)
    struct timespec t = {.tv_nsec = 0, .tv_sec = 0};
    struct timespec r = {.tv_nsec = 0, .tv_sec = 0};
    while(ms >= 1000){
        t.tv_sec++;
        ms -= 1000;
    }
    t.tv_nsec = ms * 1000000;

    while(1){
        if(nanosleep(&t, &r) == 0){
            // Success
            break;
        }else{
            // Interrupted. New time to sleep is remaining
            // Loop to continue sleep
            t = r;
        }
    }
}

#endif // CONTROL_BOARD_SIM_LINUX || CONTROL_BOARD_SIM_MACOS

#if defined(CONTROL_BOARD_SIM_WIN)

#include <windows.h>

// SimCB
void delay_init(void){}

void delay_us(unsigned int us){
    // From https://stackoverflow.com/questions/5801813/c-usleep-is-obsolete-workarounds-for-windows-mingw/11470617
    // Not good for longer wait times
    HANDLE timer; 
    LARGE_INTEGER ft; 

    ft.QuadPart = -(10*us); // Convert to 100 nanosecond interval, negative value indicates relative time

    timer = CreateWaitableTimer(NULL, TRUE, NULL); 
    SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0); 
    WaitForSingleObject(timer, INFINITE); 
    CloseHandle(timer); 
}

void delay_ms(unsigned int ms){
    Sleep((DWORD)ms);
}

#endif // CONTROL_BOARD_SIM_WIN