#pragma once

#include <stdint.h>


/**
 * Initialize communication with PC via USB
 */
void usb_init(void);

/**
 * Task to handle USB events
 * STARTED AS A FREERTOS TASK
 */
void usb_device_task(void *argument);

/**
 * Write some data to the PC
 * @param data Data to write
 * @param len Length of data to write
 */
void usb_write(uint8_t *data, unsigned int len);

/**
 * Write a string (null terminated) to the PC
 * @param msg String to write
 */
void usb_writestr(const char *msg);
