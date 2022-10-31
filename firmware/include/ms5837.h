/**
 * BNO055 Depth Sensor Driver
 * ONLY SUPPORTS 30BA variant as of now
 * Could be adapted later (changes calculations)
 * 
 * @file ms5837.h
 * @author Marcus Behel
 */

#include <util.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

extern i2c_trans ms5837_trans;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Initialize MS5837
 */
bool ms5837_init(void);

/**
 * Call from main when i2c transaction done
 */
void ms5837_i2c_done(void);

/**
 * Call from main when delay done
 */
void ms5837_delay_done(void);

/**
 * Get current depth sensor data
 */
float ms5837_get(void);

/**
 * Trigger a data read from the depth sensor
 */
void ms5837_read(void);

/**
 * Check if the depth sensor is connected (and a valid sensor)
 * @return true once connected. Otherwise false.
 */
bool ms5837_connected(void);

/**
 * Reset the state machine and sensor
 */
void ms5837_reset(void);

