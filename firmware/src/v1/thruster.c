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

#include <framework.h>
#include <thruster.h>
#include <debug.h>
#include <string.h>
#include <stdlib.h>
#include <led.h>
#include <FreeRTOS.h>
#include <task.h>

void thruster_init(void){
    float *zero_speeds = (float[]){0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    thruster_set(zero_speeds);
    TCC0_PWMStart();
    TCC1_PWMStart();
}

void thruster_set(float *speeds){
    // PWM configured on TCC0 and TCC1 for thrusters
    // PWM setup and clocks configured in generated code
    // Timer count rate = 3MHz = 3 counts / us
    // PWM frequency = 3MHz / 6000 (period reg) = 500Hz
    // Supported ESCs use pulses from 1100us to 1900us
    // 1500us pulse = 0% speed
    // 1100us pulse = -100% speed
    // 1900us pulse = 100% speed
    #define COUNT_PER_US                3
    #define PULSE_WIDTH_US(speed)       ((int)(400 * speed) + 1500)
    
    // THR1 = TCC0[2]
    while(!TCC0_PWM24bitDutySet(TCC0_CHANNEL2, PULSE_WIDTH_US(speeds[0]) * COUNT_PER_US));

    // THR2 = TCC0[3]
    while(!TCC0_PWM24bitDutySet(TCC0_CHANNEL3, PULSE_WIDTH_US(speeds[1]) * COUNT_PER_US));

    // THR3 = TCC0[1]
    while(!TCC0_PWM24bitDutySet(TCC0_CHANNEL1, PULSE_WIDTH_US(speeds[2]) * COUNT_PER_US));

    // THR4 = TCC0[0]
    while(!TCC0_PWM24bitDutySet(TCC0_CHANNEL0, PULSE_WIDTH_US(speeds[3]) * COUNT_PER_US));

    // THR5 = TCC1[3]
    while(!TCC1_PWM24bitDutySet(TCC1_CHANNEL3, PULSE_WIDTH_US(speeds[4]) * COUNT_PER_US));

    // THR6 = TCC1[2]
    while(!TCC1_PWM24bitDutySet(TCC1_CHANNEL2, PULSE_WIDTH_US(speeds[5]) * COUNT_PER_US));

    // THR7 = TCC1[1]
    while(!TCC1_PWM24bitDutySet(TCC1_CHANNEL1, PULSE_WIDTH_US(speeds[6]) * COUNT_PER_US));

    // THR8 = TCC1[0]
    while(!TCC1_PWM24bitDutySet(TCC1_CHANNEL0, PULSE_WIDTH_US(speeds[7]) * COUNT_PER_US));
}
