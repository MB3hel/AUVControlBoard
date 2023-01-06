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

#include <pid.h>


float pid_calculate(pid_controller_t *pid, float curr_err){
    #define MIN(a, b)       ((a < b) ? a : b)
    #define MAX(a, b)       ((a > b) ? a : b)

    // Start with feedforward gain
    float output = pid->kF * pid->setpoint;

    // Proportional gain
    output += pid->kP * curr_err;

    // Integral gain
    pid->integral += curr_err;
    output += pid->kI * pid->integral;

    // Derivative gain
    output += pid->kD * (curr_err - pid->last_error);
    pid->last_error = curr_err;

    // Limit output range
    return MAX(pid->min, MIN(output, pid->max));
}
