#include <framework.h>
#include <i2c.h>
#include <FreeRTOS.h>
#include <semphr.h>

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


void i2c_init(void){
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
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);

    if(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
        return false;

    if(trans->write_count > 0){
        // Perform write first.
        
        // TODO: No stop after write if data will be read.
        //       This can be done using Seq operations

        // Perform write
        HAL_I2C_Master_Transmit_IT(&hi2c1, 
            trans->address, 
            trans->write_buf, 
            trans->write_count);

        // Wait for write to finish
        xSemaphoreTake(i2c_done_signal, portMAX_DELAY);
        
        // If write fails, or no read phase, exit here
        if(!i2c_success || trans->read_count == 0){
            // Must store copy of i2c_success before unlocking
            // Otherwise it is possible it would be modified again
            // before return actually happens
            bool ret = i2c_success;
            xSemaphoreGive(i2c_mutex);
            return ret;
        }
    }

    if(trans->read_count > 0){
        // Perform read. Read always ends with STOP, so no need for Seq operations
        HAL_I2C_Master_Receive_IT(&hi2c1,
            trans->address,
            trans->read_buf,
            trans->read_count);
        xSemaphoreGive(i2c_mutex);

        // Wait for read to finish
        xSemaphoreTake(i2c_done_signal, portMAX_DELAY);

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

