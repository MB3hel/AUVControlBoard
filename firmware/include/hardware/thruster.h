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

#include <stdint.h>


// ESCs typically operate with pulses between 1ms and 2ms (1000us and 2000us)
// and typically need to be updated at least once every 20ms (50Hz PWM)
// Many support faster updates though (100-200Hz seems common)
typedef struct{
    // PWM period in us (determines PWM frequency = update rate of ESCs)
    // Note that this may not exceed 21845= 2^16 / 3 b/c PWM is 16-bit with 3MHz signal (3us/count)
    uint16_t pwm_period;

    // Zero speed for ESC (usually 1500us)
    uint16_t pwm_zero;

    // Speed range above / below zero
    // full reverse speed = pwm_zero - pwm_range
    // full forward speed = pwm_zero + pwm_range
    uint16_t pwm_range;
} thr_params_t;



/**
 * Initialize thruster control
 */
void thruster_init(void);

/**
 * Set parameters for thrusters. These must be set before thruster PWM will actually
 * be generated.
 */
void thrusters_config(thr_params_t p);

/**
 * Set thruster speeds
 * @param speeds An array of 8 floats between -1.0 and 1.0
 *               Each corresponds to a thruster speed for thruster index+1
 */
void thruster_set(float *speeds);

