
/**
 * @file i2c0.h
 * 
 * Implementation for i2c0 asynchronous master with queued transactions
 * 
 * @author Marcus Behel
 */


#pragma once

#include <atmel_start.h>
#include <stdint.h>
#include <stdbool.h>
#include <util.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Initialize i2c0 bus
 */
void i2c0_init(void);

/**
 * Handle operations for i2c0 bus
 */
void i2c0_process(void);

/**
 * Queue a transaction on the i2c0 bus
 * Transaction consists of a write followed by a read to a device with a particular address
 * Transaction is performed asynchronously
 * @param trans Pointer to the transaction to perform. Note that the object pointed to 
 * must remain valid and not be modified until the transaction is completed 
 * (trans->done is true).
 */
void i2c0_perform(i2c_trans *trans);

/**
 * Queue a transaction and block until it finishes
 */
void i2c0_perform_block(i2c_trans *trans);
