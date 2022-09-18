
#pragma once

#include <atmel_start.h>
#include <stdint.h>
#include <stdbool.h>


////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////

/**
 * Initialize UART via USB CDC device
 */
bool usbuart_init(void);

/**
 * Write data to the device on the other end (buffers, does not write yet)
 * @param data Data to write
 * @param len Length of the data array
 * @return Number of bytes copied to write buffer
 */
unsigned int usbuart_write(uint8_t *data, unsigned int len);

/**
 * Read data from the device on the other end (reads from buffer)
 * @param data Buffer to read data into
 * @param len Size of the read buffer (max number of bytes to read)
 * @return Number of bytes read
 */
unsigned int usbuart_read(uint8_t *data, unsigned int len);
