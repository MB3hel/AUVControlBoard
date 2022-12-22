#include <framework.h>
#include <i2c.h>
#include <FreeRTOS.h>
#include <portmacro.h>
#include <semphr.h>
#include <delay.h>


static SemaphoreHandle_t i2c_mutex;
static SemaphoreHandle_t i2c_done_signal;


static void i2c_done_callback(uintptr_t contextHandle){
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
    delay_ms(1);

    // Need to re-initialize I2C SERCOM after doing this or it won't work
    // May have to do with I2C SERCOM hardware getting in a bad state when pinmux changed
    // and there are no longer pullup resistors
    SERCOM2_I2C_Initialize();
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
