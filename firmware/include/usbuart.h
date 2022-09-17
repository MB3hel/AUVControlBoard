
#pragma once

#include <atmel_start.h>
#include <stdint.h>
#include <stdbool.h>

////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////

extern bool usbuart_initialized;


////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////

/**
 * Initialize UART via USB CDC device
 */
void usbuart_init(void);

/**
 * Write data to the device on the other end
 * @param data Data to write
 * @param len Length of the data array
 */
void usbuart_write(uint8_t *data, size_t len);

/**
 * Read data from the device on the other end
 * @param data Buffer to read data into
 * @param len Size of the read buffer
 * @return Number of bytes read
 */
size_t usbuart_read(uint8_t *data, size_t len);
