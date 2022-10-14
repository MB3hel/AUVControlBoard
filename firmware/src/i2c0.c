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

#define QUEUE_SIZE          6

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static volatile uint8_t state;
static volatile uint8_t result;

// Transaction queue (implemented as ring buffer)
// next = index to insert at next
// current = index of current transaction
// count = number of items in queue
static i2c_trans *queue[QUEUE_SIZE];
static uint16_t next, current, count;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void cb_tx_done(struct i2c_m_async_desc *const i2c){
    state = STATE_READ;
    FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);

    // NOTE: Not sure why this needs to be cleared in user callback
    // But not doing so results in interrupt repeatedly occurring
    // Looking in hal_i2c_m_async and hpl_sercom, it looks like this flag is
    // never cleared. My guess is that this is an SAMD51 specific ASF4 bug...
    hri_sercomi2cm_clear_INTFLAG_reg(I2C.device.hw, SERCOM_I2CM_INTFLAG_MB);
}

static void cb_rx_done(struct i2c_m_async_desc *const i2c){
    state = STATE_IDLE;
    result = I2C_STATUS_SUCCESS;
    FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);

    // NOTE: Not sure why this needs to be cleared in user callback
    // But not doing so results in interrupt repeatedly occurring
    // Looking in hal_i2c_m_async and hpl_sercom, it looks like this flag is
    // never cleared. My guess is that this is an SAMD51 specific ASF4 bug...
    hri_sercomi2cm_clear_INTFLAG_reg(I2C.device.hw, SERCOM_I2CM_INTFLAG_SB);
}

static void cb_error(struct i2c_m_async_desc *const i2c, int32_t error){
    state = STATE_IDLE;
    result = I2C_STATUS_ERROR;
    FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);
}

void i2c0_init(void){
    // Initialize queue indices
    current = 0;
    count = 0;
    next = 0;

    // Initialize I2C
    state = STATE_IDLE;

    // NOTE: Not sure why this needs to be cleared manually. Should really
    // be handled by i2c_m_async_enable to prevent rapid interrupt slowing 
    // everything. Looks like a bug in ASF4...
    // Clears all interrupt flag bits
    hri_sercomi2cm_clear_INTFLAG_reg(I2C.device.hw, 0xFF);

    i2c_m_async_enable(&I2C);
    i2c_m_async_register_callback(&I2C, I2C_M_ASYNC_TX_COMPLETE, (FUNC_PTR)cb_tx_done);
    i2c_m_async_register_callback(&I2C, I2C_M_ASYNC_RX_COMPLETE, (FUNC_PTR)cb_rx_done);
    i2c_m_async_register_callback(&I2C, I2C_M_ASYNC_ERROR, (FUNC_PTR)cb_error);
}

void i2c0_process(void){
    struct io_descriptor *io;
    struct _i2c_m_msg msg;
    i2c_m_async_get_io_descriptor(&I2C_0, &io);

    switch(state){
    case STATE_WRITE:
        // Set address
        i2c_m_async_set_slaveaddr(&I2C, queue[current]->address, I2C_M_SEVEN);

        if(queue[current]->write_count > 0){
            // Start write operation
            msg.addr = queue[current]->address;
            msg.buffer = queue[current]->write_buf;
            msg.len = queue[current]->write_count;
            msg.flags = 0;
            if(queue[current]->read_count == 0)
                msg.flags = I2C_M_STOP;
            i2c_m_async_transfer(&I2C, &msg);
        }else{
            // Nothing to write, skip to read state
            state = STATE_READ;
            FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);
        }
        break;
    case STATE_READ:
        if(queue[current]->read_count > 0){
            // Start read operation
            msg.addr = queue[current]->address;
            msg.buffer = queue[current]->read_buf;
            msg.len = queue[current]->read_count;
            msg.flags = I2C_M_RD | I2C_M_STOP;
            i2c_m_async_transfer(&I2C, &msg);
        }else{
            // Nothing to read, skip to idle state
            state = STATE_IDLE;
            FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);
        }
        break;
    case STATE_IDLE:
        // Set result and flag
        queue[current]->status = result;
        FLAG_SET(flags_main, FLAG_MAIN_I2C0_DONE);

        // Adjust queue vars
        current++;
        if(current == QUEUE_SIZE)
            current = 0;
        count--;

        // Start next transaction if queue not empty
        if(count > 0){
            state = STATE_WRITE;
            FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);
        }
        break;
    }
}

void i2c0_perform(i2c_trans *trans){
    // Add transaction to queue (cannot just start or it may interrupt a current transaction)
    i2c0_enqueue(trans);

    // Wait for this transaction to finish (anything else in the queue will run first)
    while(trans->status == I2C_STATUS_BUSY){
        if(FLAG_CHECK(flags_main, FLAG_MAIN_I2C0_PROC)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_I2C0_PROC);
            i2c0_process();
        }else if(FLAG_CHECK(flags_main, FLAG_MAIN_10MS)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_10MS);
            timers_wdt_feed();
        }
    }
}

void i2c0_enqueue(i2c_trans *trans){
    // This should never happen
    // If it does, either queue is too small or i2c is too slow
    if(count == QUEUE_SIZE)
        return;
    
    // Add trans to queue
    trans->status = I2C_STATUS_BUSY;
    queue[next] = trans;
    next++;
    if(next == QUEUE_SIZE)
        next = 0;
    count++;

    // If idle, start this transaction now
    if(state == STATE_IDLE){
        state = STATE_WRITE;
        FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);
    }
}
