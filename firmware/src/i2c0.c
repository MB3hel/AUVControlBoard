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

#define QUEUE_SIZE          16                                  // Size of I2C transaction queue

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static uint8_t curr_state, next_state;                          // Current state and next state
static struct io_descriptor *io;                                // Io descriptor

// Queue of messages (implemented as circular buffer)
// widx is index to insert next item (write)
// ridx is index of next item to remove (read)
// count is number of items currently in queue
static i2c_trans *trans_queue[QUEUE_SIZE];
static uint32_t trans_queue_widx;
static uint32_t trans_queue_ridx;
static uint32_t trans_queue_count;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void cb_tx_complete(struct i2c_m_async_desc *const i2c){
    // Called when all bytes have been written from write phase
    // Now done with write phase
    next_state = STATE_READ;
    FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);
}

static void cb_rx_complete(struct i2c_m_async_desc *const i2c){
    // Called when all bytes have been read from read phase
    // Now done with read phase
    trans_queue[trans_queue_ridx]->status = I2C_STATUS_SUCCESS;
    next_state = STATE_IDLE;
    FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);
}

static void cb_err(struct i2c_m_async_desc *const i2c){
    // Error occurred during transaction. Set status and move to idle state
    trans_queue[trans_queue_ridx]->status = I2C_STATUS_ERROR;
    next_state = STATE_IDLE;
    FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);
}

void i2c0_init(void){
    // Queue initialization
    trans_queue_widx = 0;
    trans_queue_ridx = 0;
    trans_queue_count = 0;

    i2c_m_async_get_io_descriptor(&I2C, &io);                   // Store for later
    i2c_m_async_enable(&I2C);                                   // Enable bus (ASF layer)
    i2c_m_async_register_callback(&I2C, 
            I2C_M_ASYNC_TX_COMPLETE, 
            (FUNC_PTR)&cb_tx_complete);                         // Register tx callback
    i2c_m_async_register_callback(&I2C, 
            I2C_M_ASYNC_RX_COMPLETE, 
            (FUNC_PTR)&cb_rx_complete);                         // Register rx callback
    i2c_m_async_register_callback(&I2C,
            I2C_M_ASYNC_ERROR,
            (FUNC_PTR)&cb_err);                                 // Register error callback
    
    // State initialization
    curr_state = STATE_IDLE;
    next_state = STATE_IDLE;
}

void i2c0_process(void){
    if(curr_state == next_state)
        // No state change, so no actions necessary
        return;

    // Save for switch / case comparison
    uint8_t ns = next_state;

    // Do this before switch / case because it might change state (setting next_state)
    curr_state = next_state;

    switch(ns){
    case STATE_WRITE:
        // Switching to write state once no longer idle
        // ridx points to current transaction
        // Start the current transaction's write phase
        // If no write phase, move to read phase immediately
        if(trans_queue[trans_queue_ridx]->write_count > 0){
            // Start the actual write
            // Callback transitions to read state when write finishes
            io_write(io, trans_queue[trans_queue_ridx]->write_buf, trans_queue[trans_queue_ridx]->write_count);
        }else{
            // Same actions as would be take if write phase had completed
            next_state = STATE_READ;
            FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);
        }
        break;
    case STATE_READ:
        // Switching to read state after write finished
        // ridx points to current transaction
        // Start the current transaction's read phase
        // If no read phase, move to idle phase immediately
        if(trans_queue[trans_queue_ridx]->read_count > 0){
            // Start the actual read
            // Callback transitions to idle state when read finishes
            io_read(io, trans_queue[trans_queue_ridx]->read_buf, trans_queue[trans_queue_ridx]->read_count);
        }else{
            // Same actions as would be take if read phase had completed
            trans_queue[trans_queue_ridx]->status = I2C_STATUS_SUCCESS;
            next_state = STATE_IDLE;
            FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);
        }
        break;
    case STATE_IDLE:
        // Switching to idle state after read finished
        // ridx points to the current transaction (the one that just finished)
        // Remove it from the queue
        trans_queue_ridx++;
        if(trans_queue_ridx > QUEUE_SIZE)
            trans_queue_ridx = 0;
        trans_queue_count--;
        if(trans_queue_count > 0){
            // Something else in queue
            // Transition to write state
            next_state = STATE_IDLE;
            FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);
        }
        break;
    }
}

void i2c0_perform(i2c_trans *trans){
    // This should never happen. If it does, queue probably needs to be larger
    // or the i2c bus is too slow for what is being done
    if(trans_queue_count == QUEUE_SIZE)
        return;
    
    // Add transaction to the queue
    trans->status = I2C_STATUS_BUSY;
    trans_queue[trans_queue_widx] = trans;
    trans_queue_widx++;
    if(trans_queue_widx >= QUEUE_SIZE)
        trans_queue_widx = 0;
    trans_queue_count++;

    // If idle (and no state transition already pending), start next write now
    // Write will actually be started by process (hence the flag set)
    if(curr_state == STATE_IDLE && next_state == STATE_IDLE){
        next_state = STATE_IDLE;
        FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);
    }
}
