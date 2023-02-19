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

#pragma once

#include <stdbool.h>


typedef struct{
    // Gains
    float kP;
    float kI;
    float kD;

    // Output limits
    float min;
    float max;

    // True to negate output
    bool invert;

    // State info (zero to reset)
    float integral;
    float last_error;
} pid_controller_t;



// Reset a PID controller
#define PID_RESET(pid)          (pid).integral = 0; (pid).last_error = 0


/**
 * Calculate current PID output
 * @param pid PID controller
 * @param curr_error Current error
 */
float pid_calculate(pid_controller_t *pid, float curr_error);
