/**
 * @file i2c0.c
 * @author Marcus Behel
 */

#include <i2c0.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define I2C                 I2C_0                               // Which ASF I2C object to use

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static bool idle;                                               // Current state (idle or busy)
static struct io_descriptor *io;                                // Io descriptor
static i2c_trans *curr_trans;                                   // Current transaction

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void cb_tx_complete(struct i2c_m_async_desc *const i2c){
    // Called when all bytes have been written from write phase
    // Now move to read state (if any)
    if(curr_trans->read_count > 0){
        // Start read phase
        io_read(io, curr_trans->read_buf, curr_trans->read_count);
    }else{
        // No read phase, transaction done
        idle = true;
    }
}

static void cb_rx_complete(struct i2c_m_async_desc *const i2c){
    // Called when all bytes have been read from read phase
    // Read always happens after write phase
    // Now done with transaction
    idle = true;
}

void i2c0_init(void){
    i2c_m_async_get_io_descriptor(&I2C, &io);                   // Store for later
    i2c_m_async_enable(&I2C);                                   // Enable bus (ASF layer)
    i2c_m_async_register_callback(&I2C, 
            I2C_M_ASYNC_TX_COMPLETE, &cb_tx_complete);          // Register tx callback
    i2c_m_async_register_callback(&I2C, 
            I2C_M_ASYNC_RX_COMPLETE, &cb_rx_complete);          // Register rx callback
    idle = true;                                                // Idle on start
}

void i2c0_perform(i2c_trans *trans){
    if(!idle)
        // Currently performing transaction; cannot interrupt
        return;

    curr_trans = trans;                                         // Update current transaction pointer
    i2c_m_async_set_slaveaddr(&I2C, 
            trans->address, I2C_M_SEVEN);                       // Set address for transaction (7-bit addr mode)
    
    if(trans->write_count > 0){
        // Start write
        idle = false;
        io_write(io, trans->write_buf, trans->write_count);
    }else if(trans->read_count > 0){
        // No write phase, but a read phase
        // Start read
        idle = false;
        io_read(io, trans->read_buf, trans->read_count);
    }else{
        // Nothing to do in this transaction
        // remains idle
    }
}

bool i2c0_idle(void){
    return idle;
}