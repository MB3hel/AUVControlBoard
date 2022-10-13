
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
 * Call when i2c0 needs state change (I2C0_PROC flag set)
 */
void i2c0_process(void);

/**
 * Perform a transaction and BLOCK UNTIL DONE.
 */
void i2c0_perform(i2c_trans *trans);

/**
 * Add a new transaction to the queue
 * POINTER MUST REMAIN VALID UNTIL STATUS IS NOT BUSY!
 */
void i2c0_enqueue(i2c_trans *trans);
