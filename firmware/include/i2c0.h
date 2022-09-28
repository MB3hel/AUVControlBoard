
/**
 * @file i2c0.h
 * 
 * Implementation for i2c0 asynchronous master state machine
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
 * Perform a transaction on the i2c0 bus
 * Transaction consists of a write followed by a read to a device with a particular address
 * Transaction is performed asynchronously
 * This function should not be called unless idle
 * @param trans Pointer to the transaction to perform.
 */
void i2c0_perform(i2c_trans *trans);

/**
 * Check if i2c0 is idle
 * @return true if idle (no transaction in progress)
 * @return fals if not idle (transaction in progress)
 */
bool i2c0_idle(void);
