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
#include <i2c.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <delay.h>

extern I2C_HandleTypeDef hi2c1;

static SemaphoreHandle_t i2c_mutex;
static SemaphoreHandle_t i2c_done_signal;
static volatile bool i2c_success = false;


void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef * hi2c){
    // Signal that transaction is now finished
    // This is ALWAYS run from IRQ handler (by inspection of generated hal)
    static BaseType_t xHigherPriorityTaskWoken;
    i2c_success = true;
    xSemaphoreGiveFromISR(i2c_done_signal, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef * hi2c){
    // Signal that transaction is now finished
    // This is ALWAYS run from IRQ handler (by inspection of generated hal)
    static BaseType_t xHigherPriorityTaskWoken;
    i2c_success = true;
    xSemaphoreGiveFromISR(i2c_done_signal, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
    // Signal that transaction is now finished
    // This is ALWAYS run from IRQ handler (by inspection of generated hal)
    static BaseType_t xHigherPriorityTaskWoken;
    i2c_success = false;
    xSemaphoreGiveFromISR(i2c_done_signal, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c){
    asm("nop");
}

static void i2c_fix_sda_low(void){
    // Works by setting SDA and SCL pins into GPIO mode and bit-banging clock
    // cycles until the SDA line no longer reads as low
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Set pins into GPIO mode (generated code will have put them in SDA1 and SCL1 modes)
    // SCL (PB8) as output
    // SDA (PB9) as input
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    delay_ms(5);

    // Bit-bang clock cycles while SDA remains low (~100kHz clock frequency)
    // Limit max cycles to ensure this is never an infinite loop (eg if external pullups missing)
    // In reality, should never take more than 10 clock cycles
    unsigned int cycles = 0;
    while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET){
        if(cycles++ > 100)
            break;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        delay_us(10);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        delay_us(10);
    }

    // Set pins to I2C mode (SDA and SCL modes; same as generated code does)
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    delay_ms(5);
}

static void i2c_reinit(void){
    HAL_I2C_DeInit(&hi2c1);
    delay_us(100);
    HAL_I2C_Init(&hi2c1);
    NVIC_SetPriority(I2C1_EV_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(I2C1_ER_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);
}

void i2c_init(void){

    // Sometimes (when device is reset at wrong time) a slave may be holding SDA low
    // Best option in this case is to send clock pulses until it releases it
    // This could be fixed in the case of the BNO055 using the reset pin to reset the IMU
    // however, this cannot be done for the depth sensor. Thus, this method is relied on
    // for both devices instead.
    i2c_fix_sda_low();

    // Mutex used to ensure i2c API is thread-safe
    i2c_mutex = xSemaphoreCreateMutex();

    // Used to block perform until transfer complete
    // Value after create is zero (must be given before take succeeds)
    i2c_done_signal = xSemaphoreCreateBinary();

    // I2C IRQ_Handler priority needs to be such that FreeRTOS functions can be called
    // IRQ handlers run the callback functions, which use the semaphore
    NVIC_SetPriority(I2C1_EV_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(I2C1_ER_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);

    // Framework init initializes I2C1
    // Clock and pin config also handled by generator project
}

bool i2c_perform(i2c_trans *trans){
    HAL_StatusTypeDef status;


    // When waiting on semaphore signaling i2c finish from interrupt
    // a timeout is used to ensure that even if something is configured very wrong (such that transmit appears to start
    // but interrupt never occurs), this won't hold the i2c mutex forever
    // The timeout is far longer than the operation should take.
    // With a 100kHz clock, each bit takes 10us to transmit. Thus expected time is 10us*(read_count+write_count)*8
    // However, clock stretching can occur. To account for this, allow 50us between bytes. Thus
    // max_time = 50us*(read_count+write_count) + 10us*(read_count+write_count)*8 = 130us*(read_count+write_count)
    // For a safety margin, round up to 200us per byte. Thus timeout in ms = ceil(200*(read_count+write_count)/1000)
    // = ceil((read_count+write_count)/5) = floor((read_count+write_count+4)/5) 
    // = integer division of (read_count+write_count+4) by 5
    // Then for extra margin, add 5ms
    #define TIMEOUT_FOR_COUNT(count)        ((((uint32_t)(count + 4)) / 5) + 5)


    // I2C runs at 100kHz clock
    // A transaction with 64 bytes read and write each would be 128*8=1024 bits
    // 1/100kHz = 10us per bit
    // 1024 * 10 = 10240us bit transfer time = 1.024ms
    // Assume some clock stretching and delays with ACK up to 30us per byte
    // 128*30 = 3.840ms
    // Thus a "large" transaction should finish within 5ms
    // Assume it is possible for a few to be queued up.
    // Thus wait for at most 25ms (5 queued large transactions)
    // If this fails, something is probably stuck and will never release the mutex
    // This is also a small enough amount of time to not fully break most threads
    // calling this function
    if(xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(25)) == pdFALSE)
        return false;

    if(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){
        xSemaphoreGive(i2c_mutex);
        return false;
    }

    if(trans->write_count > 0 && trans->read_count > 0){
        // Write then read (repeated start between)
        
        // Perform write (no stop after)
        status = HAL_I2C_Master_Seq_Transmit_IT(&hi2c1, 
            trans->address << 1, 
            trans->write_buf, 
            trans->write_count,
            I2C_FIRST_FRAME);

        if(status != HAL_OK){

            if(__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET){
                // This occurs when timeout waiting for the bus to no longer be in busy state
                // The only reason this would be in a busy state is if another master were using the bus
                // This is not possible in this system. Thus, this happens due to noise on I2C lines
                // usually from hot-plugging sensors. Thus, this should be ignored by manually clearing the flag.
                // BUSY flag cannot be written (it is only able to be cleared by hardware)
                // So the only solution is to reset the I2C peripheral...
                // Unfortunately, this noise may have also caused a sensor to get stuck holding SDA low
                // Thus, need to handle that too here.
                i2c_fix_sda_low();
                i2c_reinit();
            }

            xSemaphoreGive(i2c_mutex);
            return false;
        }

        // Wait for write to finish
        if(xSemaphoreTake(i2c_done_signal, pdMS_TO_TICKS(TIMEOUT_FOR_COUNT(trans->write_count))) == pdFALSE){
            HAL_I2C_Master_Abort_IT(&hi2c1, trans->address << 1);   // Aborts transfer. Calls callback directly
            while(xSemaphoreTake(i2c_done_signal, 0) == pdTRUE);    // Zero semaphore (may have been given before abort)
            xSemaphoreGive(i2c_mutex);
            return false;
        }
        
        // If write fails, exit here
        if(!i2c_success){
            // Must store copy of i2c_success before unlocking
            // Otherwise it is possible it would be modified again
            // before return actually happens
            bool ret = i2c_success;
            xSemaphoreGive(i2c_mutex);
            return ret;
        }

        // Perform read
        status = HAL_I2C_Master_Seq_Receive_IT(&hi2c1,
            trans->address << 1,
            trans->read_buf,
            trans->read_count,
            I2C_LAST_FRAME);

        if(status != HAL_OK){

            if(__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET){
                // This occurs when timeout waiting for the bus to no longer be in busy state
                // The only reason this would be in a busy state is if another master were using the bus
                // This is not possible in this system. Thus, this happens due to noise on I2C lines
                // usually from hot-plugging sensors. Thus, this should be ignored by manually clearing the flag.
                // Unfortunately, this noise may have also caused a sensor to get stuck holding SDA low
                // Thus, need to handle that too here.
                i2c_fix_sda_low();
                i2c_reinit();
            }

            xSemaphoreGive(i2c_mutex);
            return false;
        }

        // Wait for read to finish
        if(xSemaphoreTake(i2c_done_signal, pdMS_TO_TICKS(TIMEOUT_FOR_COUNT(trans->read_count))) == pdFALSE){
            HAL_I2C_Master_Abort_IT(&hi2c1, trans->address << 1);   // Aborts transfer. Calls callback directly
            while(xSemaphoreTake(i2c_done_signal, 0) == pdTRUE);    // Zero semaphore (may have been given before abort)
            xSemaphoreGive(i2c_mutex);
            return false;
        }

        // Must store copy of i2c_success before unlocking
        // Otherwise it is possible it would be modified again
        // before return actually happens
        bool ret = i2c_success;
        xSemaphoreGive(i2c_mutex);
        return ret;
    }else if(trans->read_count > 0){
        // Perform read only.
        status = HAL_I2C_Master_Receive_IT(&hi2c1,
            trans->address << 1,
            trans->read_buf,
            trans->read_count);

        if(status != HAL_OK){

            if(__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET){
                // This occurs when timeout waiting for the bus to no longer be in busy state
                // The only reason this would be in a busy state is if another master were using the bus
                // This is not possible in this system. Thus, this happens due to noise on I2C lines
                // usually from hot-plugging sensors. Thus, this should be ignored by manually clearing the flag.
                // Unfortunately, this noise may have also caused a sensor to get stuck holding SDA low
                // Thus, need to handle that too here.
                i2c_fix_sda_low();
                i2c_reinit();
            }

            xSemaphoreGive(i2c_mutex);
            return false;
        }

        // Wait for read to finish
        if(xSemaphoreTake(i2c_done_signal, pdMS_TO_TICKS(TIMEOUT_FOR_COUNT(trans->read_count))) == pdFALSE){
            HAL_I2C_Master_Abort_IT(&hi2c1, trans->address << 1);   // Aborts transfer. Calls callback directly
            while(xSemaphoreTake(i2c_done_signal, 0) == pdTRUE);    // Zero semaphore (may have been given before abort)
            xSemaphoreGive(i2c_mutex);
            return false;
        }

        // Must store copy of i2c_success before unlocking
        // Otherwise it is possible it would be modified again
        // before return actually happens
        bool ret = i2c_success;
        xSemaphoreGive(i2c_mutex);
        return ret;
    }else if(trans->write_count > 0){
        // Perform write only
        status = HAL_I2C_Master_Transmit_IT(&hi2c1,
            trans->address << 1,
            trans->write_buf,
            trans->write_count);

        if(status != HAL_OK){

            if(__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET){
                // This occurs when timeout waiting for the bus to no longer be in busy state
                // The only reason this would be in a busy state is if another master were using the bus
                // This is not possible in this system. Thus, this happens due to noise on I2C lines
                // usually from hot-plugging sensors. Thus, this should be ignored by manually clearing the flag.
                // Unfortunately, this noise may have also caused a sensor to get stuck holding SDA low
                // Thus, need to handle that too here.
                i2c_fix_sda_low();
                i2c_reinit();
            }

            xSemaphoreGive(i2c_mutex);
            return false;
        }

        // Wait for write to finish
        if(xSemaphoreTake(i2c_done_signal, pdMS_TO_TICKS(TIMEOUT_FOR_COUNT(trans->write_count))) == pdFALSE){
            HAL_I2C_Master_Abort_IT(&hi2c1, trans->address << 1);   // Aborts transfer. Calls callback directly
            while(xSemaphoreTake(i2c_done_signal, 0) == pdTRUE);    // Zero semaphore (may have been given before abort)
            xSemaphoreGive(i2c_mutex);
            return false;
        }

        // Must store copy of i2c_success before unlocking
        // Otherwise it is possible it would be modified again
        // before return actually happens
        bool ret = i2c_success;
        xSemaphoreGive(i2c_mutex);
        return ret;
    }

    // Only gets here if empty transaction
    xSemaphoreGive(i2c_mutex);
    return true;
}
