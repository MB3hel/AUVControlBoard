/*
 * Copyright 2022 Marcus Behel
 * 
 * This file is part of AUVControlBoard-Firmware.
 * 
 * AUVControlBoard-Firmware is free software: you can redistribute it and/or modify it under the terms of the GNU 
 * General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 * 
 * AUVControlBoard-Firmware is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even 
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for 
 * more details.
 * 
 * You should have received a copy of the GNU General Public License along with AUVControlBoard-Firmware. If not, see 
 * <https://www.gnu.org/licenses/>. 
 * 
 */

#pragma once

// -------------------------------------------------------------------------------------------------
// Board Specific Configuration
// -------------------------------------------------------------------------------------------------

#define BOARD_TUD_RHPORT        0                               // RHPort number used for device
#define BOARD_TUD_MAX_SPEED     OPT_MODE_FULL_SPEED             // RHPort max speed supported for device


// -------------------------------------------------------------------------------------------------
// Common Configuration
// -------------------------------------------------------------------------------------------------

#ifndef CFG_TUSB_MCU
#error "Define CFG_TUSB_MCU via build flags!"
#endif

#define CFG_TUSB_OS             OPT_OS_FREERTOS                 // FreeRTOS is used
#define CFG_TUSB_DEBUG          0

#define CFG_TUD_ENABLED         1                               // Enable device stack
#define CFG_TUD_MAX_SPEED       BOARD_TUD_MAX_SPEED             // Speed for device stack

// See example configs for description of these
#define CFG_TUSB_MEM_SECTION                                    // No special mem section needed for USB DMA
#define CFG_TUSB_MEM_ALIGN      __attribute__ ((aligned(4)))    // Align memory for USB DMA


// -------------------------------------------------------------------------------------------------
// DEVICE CONFIGURATION
// -------------------------------------------------------------------------------------------------

#define CFG_TUD_ENDPOINT0_SIZE  64                              // Device endpoint size

// Device classes (changing requires descriptor changes)
#define CFG_TUD_CDC             1                               // Enable CDC class
#define CFG_TUD_MSC             0                               // Disable MSC class
#define CFG_TUD_HID             0                               // Disable HID class
#define CFG_TUD_MIDI            0                               // Disable MIDI class
#define CFG_TUD_VENDOR          0                               // Disable VENDOR class

// CDC FIFO size of TX and RX
#define CFG_TUD_CDC_RX_BUFSIZE   256
#define CFG_TUD_CDC_TX_BUFSIZE   256

// CDC Endpoint transfer buffer size, more is faster
#define CFG_TUD_CDC_EP_BUFSIZE   64
