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
