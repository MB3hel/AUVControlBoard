
/**
 * @file i2c0.h
 * 
 * Implementation for i2c0 asynchronous master with queued transactions
 * 
 * @author Marcus Behel
 */


#pragma once

#include <util.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Pointer to current transaction
// When I2C0_DONE flag is set, this points to transaction that was just finished
// It will change when i2c0_start is called
extern i2c_trans *i2c0_curr_trans;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Initialize i2c0 bus
 */
void i2c0_init(void);

/**
 * Start a transaction. Note that a STOP condition  will not occur between write and read phases
 * @param trans The transaction to start. Object pointed to must remain in scope
 *              for duration of the transaction.
 */
bool i2c0_start(i2c_trans *trans);
