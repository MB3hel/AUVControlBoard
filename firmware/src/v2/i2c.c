#include <framework.h>
#include <i2c.h>
#include <FreeRTOS.h>
#include <semphr.h>

static SemaphoreHandle_t i2c_mutex;
static SemaphoreHandle_t i2c_done_signal;


// TODO: Callbacks???


void i2c_init(void){
    // Mutex used to ensure i2c API is thread-safe
    i2c_mutex = xSemaphoreCreateMutex();

    // Used to block perform until transfer complete
    // Value after create is zero (must be given before take succeeds)
    i2c_done_signal = xSemaphoreCreateBinary();

    // I2C IRQ_Handler priority needs to be such that FreeRTOS functions can be called
    // IRQ Handler runs the done callback, which gives semaphore
    // TODO

    // Framework init initializes SERCOM2 as I2C
    // Clock and pin config also handled by generator project

    // Register callback
    // TODO
}

bool i2c_perform(i2c_trans *trans){
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);

    // TODO
    
    xSemaphoreGive(i2c_mutex);
}

