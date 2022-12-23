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

#include <i2c.h>
#include <FreeRTOS.h>
#include <task.h>

bool i2c_perform_retries(i2c_trans *trans, unsigned int delay_ms, unsigned int max_retires){
    for(unsigned int i = 0; i < max_retires; ++i){
        if(i2c_perform(trans))
            return true;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    return false;
}
