/**
 * USB interface using TinyUSB
 * Provides CDC interface for UART communication with PC
 * 
 * @file usb.h
 * @author Marcus Behel
 */

#pragma once

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define USB_MAX_MSG_LEN                 64                          // Max number of bytes in one message


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Initialize USB functionality
 */
void usb_init(void);

/**
 * Handle periodic usb tasks
 * This should be called frequently so TinyUSB can process events
 */
void usb_process(void);

/**
 * Get the next available message from the message queue
 * @param dest where to copy next available message (must be at least USB_MAX_MSG_LEN bytes)
 * @return number of bytes copied (0 if no message)
 */
uint32_t usb_getmsg(uint8_t *dest);

/**
 * Write a message to the connected PC via USB CDC interface
 * @param msg Message to send
 * @param len Length of message
 */
void usb_writemsg(uint8_t *msg, uint32_t len);
