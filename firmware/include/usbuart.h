
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
 * Write one byte to the device on the other end (buffers, does not write yet)
 * @param data Value to write
 * @return true On success
 * @return false On failure (write buffer full)
 */
bool usbuart_writeone(uint8_t data);

/**
 * Read data from the device on the other end (reads from buffer)
 * @param data Buffer to read data into
 * @param len Size of the read buffer (max number of bytes to read)
 * @return Number of bytes read
 */
unsigned int usbuart_read(uint8_t *data, unsigned int len);

/**
 * Read one byte from the device on the other end (reads from buffer)
 * @param data Where to store read byte
 * @return true On success
 * @return false On failure (no data to read)
 */
bool usbuart_readone(uint8_t *data);
