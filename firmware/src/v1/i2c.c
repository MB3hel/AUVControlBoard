#include <framework.h>
#include <i2c.h>
#include <FreeRTOS.h>
#include <portmacro.h>
#include <semphr.h>


static SemaphoreHandle_t i2c_mutex;
static SemaphoreHandle_t i2c_done_signal;


static void i2c_done_callback(uintptr_t contextHandle){
    // Signal that transaction is now finished
    // This is ALWAYS run from IRQ handler (by inspection of generated plib)
    static BaseType_t xHigherPriorityTaskWoken;
    xSemaphoreGiveFromISR(i2c_done_signal, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void i2c_init(void){
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
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);

    if(SERCOM2_I2C_IsBusy())
        return false;
    
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
    xSemaphoreTake(i2c_done_signal, portMAX_DELAY);

    // I2C transaction completed
    SERCOM_I2C_ERROR result = SERCOM2_I2C_ErrorGet();
    xSemaphoreGive(i2c_mutex);
    return result == SERCOM_I2C_ERROR_NONE;
}
