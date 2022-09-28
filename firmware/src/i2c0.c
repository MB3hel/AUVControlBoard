/**
 * @file i2c0.c
 * @author Marcus Behel
 */

#include <i2c0.h>
#include <flags.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define I2C                 I2C_0                               // Which ASF I2C object to use

// States
#define STATE_IDLE          0
#define STATE_WRITE         1
#define STATE_READ          2

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static uint8_t curr_state, next_state;                          // Current state and next state
static struct io_descriptor *io;                                // Io descriptor


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void cb_tx_complete(struct i2c_m_async_desc *const i2c){
    // Called when all bytes have been written from write phase
    // Now move to read state (if any)
    // TODO
}

static void cb_rx_complete(struct i2c_m_async_desc *const i2c){
    // Called when all bytes have been read from read phase
    // Read always happens after write phase
    // Now done with transaction
    // TODO
}

void i2c0_init(void){
    i2c_m_async_get_io_descriptor(&I2C, &io);                   // Store for later
    i2c_m_async_enable(&I2C);                                   // Enable bus (ASF layer)
    i2c_m_async_register_callback(&I2C, 
            I2C_M_ASYNC_TX_COMPLETE, 
            (FUNC_PTR)&cb_tx_complete);                         // Register tx callback
    i2c_m_async_register_callback(&I2C, 
            I2C_M_ASYNC_RX_COMPLETE, 
            (FUNC_PTR)&cb_rx_complete);                         // Register rx callback
    curr_state = STATE_IDLE;
    next_state = STATE_IDLE;
}

void i2c0_process(void){
    if(curr_state == next_state)
        // No state change, so no actions necessary
        return;
    
    switch(next_state){
    case STATE_WRITE:
        // Switching to write state once no longer idle
        // TODO
        break;
    case STATE_READ:
        // Switching to read state after write finished
        // TODO
        break;
    case STATE_IDLE:
        // Switching to idle state after read finished
        // TODO
        break;
    }

    curr_state = next_state;
}

void i2c0_perform(i2c_trans *trans){
    trans->done = false;
    // TODO: Add current transaction to queue
    if(curr_state == STATE_IDLE){
        // Change state and have main call process to facilitate state change
        next_state = STATE_WRITE;
        FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);
    }
}
