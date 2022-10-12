
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
 * Start a transaction (if idle)
 */
void i2c0_perform(i2c_trans *trans);

/**
 * Perform a transaction and block until it finishes
 * WARNING: DOES NOT FEED WATCHDOG!!!
 */
void i2c0_perform_blocking(i2c_trans *trans);
