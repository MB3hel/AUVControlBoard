/**
 * @file i2c0.c
 * @author Marcus Behel
 */

#include <i2c0.h>
#include <flags.h>
#include <timers.h>
#include <atmel_start.h>
#include <dotstar.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define I2C                 I2C_0                               // Which ASF I2C object to use

// States
#define STATE_IDLE          0                                   // Idle, doing nothing
#define STATE_WRITE         1                                   // Write a byte
#define STATE_READ          2                                   // Read a byte

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static uint8_t state;
static i2c_trans *curr_trans;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void cb_tx_done(struct i2c_m_async_desc *const i2c){
    if(curr_trans->read_count == 0){
        state = STATE_IDLE;
        FLAG_SET(flags_main, FLAG_MAIN_I2C0_DONE);
    }else{
        state = STATE_READ;
        FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);
    }
}

void cb_rx_done(struct i2c_m_async_desc *const i2c){
    curr_trans->status = I2C_STATUS_SUCCESS;
    state = STATE_IDLE;
    FLAG_SET(flags_main, FLAG_MAIN_I2C0_DONE);
}

void cb_error(struct i2c_m_async_desc *const i2c){
    curr_trans->status = I2C_STATUS_ERROR;
    state = STATE_IDLE;
    FLAG_SET(flags_main, FLAG_MAIN_I2C0_DONE);
}

void i2c0_init(void){
    state = STATE_IDLE;
    i2c_m_async_enable(&I2C);
    i2c_m_async_register_callback(&I2C, I2C_M_ASYNC_TX_COMPLETE, (FUNC_PTR)cb_tx_done);
    i2c_m_async_register_callback(&I2C, I2C_M_ASYNC_RX_COMPLETE, (FUNC_PTR)cb_rx_done);
    i2c_m_async_register_callback(&I2C, I2C_M_ASYNC_TX_COMPLETE, (FUNC_PTR)cb_error);
}

void i2c0_process(void){
    struct io_descriptor *io;
    struct _i2c_m_msg msg;
    i2c_m_async_get_io_descriptor(&I2C_0, &io);

    switch(state){
    case STATE_WRITE:
	    msg.addr = curr_trans->address;
        msg.buffer = curr_trans->write_buf;
	    msg.len = curr_trans->write_count;
        msg.flags = 0;
        i2c_m_async_transfer(&I2C, &msg);
        break;
    case STATE_READ:
        msg.addr = curr_trans->address;
        msg.buffer = curr_trans->read_buf;
        msg.len = curr_trans->read_count;
        msg.flags = I2C_M_RD | I2C_M_STOP;
        i2c_m_async_transfer(&I2C, &msg);
        break;
    }
}

void i2c0_perform(i2c_trans *trans){
    if(state != STATE_IDLE)
        return;
    curr_trans = trans;
    curr_trans->status = I2C_STATUS_BUSY;
    i2c_m_async_set_slaveaddr(&I2C, curr_trans->address, I2C_M_SEVEN);
    state = STATE_WRITE;
    FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);
}

void i2c0_perform_blocking(i2c_trans *trans){
    i2c0_perform(trans);
    while(!FLAG_CHECK(flags_main, FLAG_MAIN_I2C0_DONE)){
        if(FLAG_CHECK(flags_main, FLAG_MAIN_I2C0_PROC)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_I2C0_PROC);
            i2c0_process();
        }
    }
    FLAG_CLEAR(flags_main, FLAG_MAIN_I2C0_DONE);
}
