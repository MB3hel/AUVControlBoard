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
#include <portmacro.h>
#include <semphr.h>
#include <delay.h>


static SemaphoreHandle_t i2c_mutex;
static SemaphoreHandle_t i2c_done_signal;


static void i2c_done_callback(uintptr_t contextHandle){
    (void)contextHandle;

    // Signal that transaction is now finished
    // This is ALWAYS run from IRQ handler (by inspection of generated plib)
    static BaseType_t xHigherPriorityTaskWoken;
    xSemaphoreGiveFromISR(i2c_done_signal, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void i2c_fix_sda_low(void){
    // Works by setting SDA and SCL pins into GPIO mode and bit-banging clock
    // cycles until the SDA line no longer reads as low

    // Set pins into GPIO mode (generated code will have put them in SDA1 and SCL1 modes)
    // SCL (PA13) as output
    // SDA (PA12) as input
    PORT_PinGPIOConfig(PORT_PIN_PA12);
    PORT_GroupInputEnable(PORT_GROUP_0, 1 << 12);
    PORT_PinGPIOConfig(PORT_PIN_PA13);
    PORT_GroupOutputEnable(PORT_GROUP_0, 1 << 13);
    delay_ms(1);

    // Bit-bang clock cycles while SDA remains low (~100kHz clock frequency)
    // Limit max cycles to ensure this is never an infinite loop (eg if external pullups missing)
    // In reality, should never take more than 10 clock cycles
    unsigned int cycles = 0;
    while((PORT_GroupRead(PORT_GROUP_0) & (1 << 12))){
        if(cycles++ > 100)
            break;
        PORT_GroupSet(PORT_GROUP_0, 1 << 13);
        delay_us(10);
        PORT_GroupClear(PORT_GROUP_0, 1 << 13);
        delay_us(10);
    }

    // Set pins to I2C mode (SDA and SCL modes; same as generated code does)
    PORT_PinPeripheralFunctionConfig(PORT_PIN_PA12, PERIPHERAL_FUNCTION_C);
    PORT_PinPeripheralFunctionConfig(PORT_PIN_PA13, PERIPHERAL_FUNCTION_C);
    delay_ms(5);

    // Need to re-initialize I2C SERCOM after doing this or it won't work
    // May have to do with I2C SERCOM hardware getting in a bad state when pinmux changed
    // and there are no longer pullup resistors
    SERCOM2_I2C_Initialize();
}

void i2c_init(){
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
    // IRQ Handler runs the done callback, which gives semaphore
    NVIC_SetPriority(SERCOM2_0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(SERCOM2_1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(SERCOM2_2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(SERCOM2_OTHER_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    // Framework init initializes SERCOM2 as I2C
    // Clock and pin config also handled by generator project

    // Register callback
    SERCOM2_I2C_CallbackRegister(i2c_done_callback, 0);
}

bool i2c_perform(i2c_trans *trans){
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

    if(SERCOM2_I2C_IsBusy()){
        if ((SERCOM2_REGS->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_BUSSTATE_Msk) == SERCOM_I2CM_STATUS_BUSSTATE(0x03U)){
            // Busy state indicates that some other master owns the bus
            // There is no other master, thus this is impossible
            // In practice, noise on the I2C lines can cause this (even seemingly small amounts of noise)
            // Thus, if this happens, the bus must be forced back into an idle state

            // This function disables, then re-enables the bus and forces it into an idle state
            // SERCOM2_I2C_TransferAbort();

            // Note that this can occur while a sensor is holding SDA low. In this case, it is necessary to re-init i2c entirely
            // including the bit-banged i2c low fix
            i2c_fix_sda_low();
            // SERCOM2_I2C_Initialize();    // Already called by i2c_fix_sda_low
        }

        xSemaphoreGive(i2c_mutex);
        return false;
    }

    // Zero semaphore (in case it has been given; sometimes happens when hotplugging sensors)
    while(xSemaphoreTake(i2c_done_signal, 0) == pdTRUE);
    
    if(trans->write_count > 0 && trans->read_count > 0){
        // Both write and read
        if(!SERCOM2_I2C_WriteRead(trans->address, trans->write_buf, trans->write_count, trans->read_buf, trans->read_count)){
            xSemaphoreGive(i2c_mutex);
            return false;
        }
    }else if(trans->write_count == 0 && trans->read_count > 0){
        // Read only
        if(!SERCOM2_I2C_Read(trans->address, trans->read_buf, trans->read_count)){
            xSemaphoreGive(i2c_mutex);
            return false;
        }
    }else if(trans->write_count > 0 && trans->read_count == 0){
        // Write only
        if(!SERCOM2_I2C_Write(trans->address, trans->write_buf, trans->write_count)){
            xSemaphoreGive(i2c_mutex);
            return false;
        }
    }else{
        // Empty transaction
        xSemaphoreGive(i2c_mutex);
        return true;
    }

    // Wait for transaction to finish
    // A timeout is used to ensure that even if something is configured very wrong (such that transmit appears to start
    // but interrupt never occurs), this won't hold the i2c mutex forever
    // The timeout is far longer than the operation should take.
    // With a 100kHz clock, each bit takes 10us to transmit. Thus expected time is 10us*(read_count+write_count)*8
    // However, clock stretching can occur. To account for this, allow 50us between bytes. Thus
    // max_time = 50us*(read_count+write_count) + 10us*(read_count+write_count)*8 = 130us*(read_count+write_count)
    // For a safety margin, round up to 200us per byte. Thus timeout in ms = ceil(200*(read_count+write_count)/1000)
    // = ceil((read_count+write_count)/5) = floor((read_count+write_count+4)/5) 
    // = integer division of (read_count+write_count+4) by 5
    // Then for extra margin, add 5ms
    unsigned int timeout_ms = (trans->read_count + trans->write_count + 4) / 5;
    if(xSemaphoreTake(i2c_done_signal, pdMS_TO_TICKS(timeout_ms + 5)) == pdFALSE){
        // Timed out while waiting for transfer done signal
        SERCOM2_I2C_TransferAbort();                            // Interrupt won't occur after this is done running
        while(xSemaphoreTake(i2c_done_signal, 0) == pdTRUE);    // Zero the semaphore (may have been given before abort)
        xSemaphoreGive(i2c_mutex);
        return false;
    }

    // I2C transaction completed
    SERCOM_I2C_ERROR result = SERCOM2_I2C_ErrorGet();

    if(result == SERCOM_I2C_ERROR_BUS){
        // Bus error occurs when hardware gets in a bad state
        // Only known solution is to re-initialize
        // Also, need to handle the case where sda is low when this happens, thus fix sda low is called
        i2c_fix_sda_low();
        // SERCOM2_I2C_Initialize();
    }

    xSemaphoreGive(i2c_mutex);
    return result == SERCOM_I2C_ERROR_NONE;
}
