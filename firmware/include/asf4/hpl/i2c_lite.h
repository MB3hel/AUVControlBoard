
/**
 * \file
 *
 * \brief I2C related functionality declaration.
 *
 * Copyright (c) 2017 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

#ifndef _I2C_LITE_H_INCLUDED
#define _I2C_LITE_H_INCLUDED

#include <compiler.h>
#include <peripheral_clk_config.h>

/**
 * \addtogroup i2c I2C driver
 *
 * \section i2c_rev Revision History
 * - v0.0.0.1 Initial Commit
 *
 *@{
 */

#ifdef __cplusplus
extern "C" {
#endif

/**             gclk_freq - (i2c_scl_freq * 10) - (gclk_freq * i2c_scl_freq * Trise)
 * BAUD + BAUDLOW = --------------------------------------------------------------------
 *                  i2c_scl_freq
 * BAUD:    register value low  [7:0]
 * BAUDLOW: register value high [15:8], only used for odd BAUD + BAUDLOW
 */
#define SERCOM2_BAUD_BAUDLOW                                                                                           \
	(((CONF_GCLK_SERCOM2_CORE_FREQUENCY - (100000 * 10)                                                                \
	   - (215 * (100000 / 100) * (CONF_GCLK_SERCOM2_CORE_FREQUENCY / 10000) / 1000))                                   \
	      * 10                                                                                                         \
	  + 5)                                                                                                             \
	 / (100000 * 10))
#ifndef SERCOM2_BAUD_RATE
#if SERCOM2_BAUD_BAUDLOW > (0xFF * 2)
#warning Requested I2C baudrate too low, please check
#define SERCOM2_BAUD_RATE 0xFF
#elif SERCOM2_BAUD_BAUDLOW <= 1
#warning Requested I2C baudrate too high, please check
#define SERCOM2_BAUD_RATE 1
#else
#define SERCOM2_BAUD_RATE                                                                                              \
	((SERCOM2_BAUD_BAUDLOW & 0x1) ? (SERCOM2_BAUD_BAUDLOW / 2) + ((SERCOM2_BAUD_BAUDLOW / 2 + 1) << 8)                 \
	                              : (SERCOM2_BAUD_BAUDLOW / 2))
#endif
#endif

/**
 * \brief Initialize usart interface
 *
 * \return Initialization status.
 */
int8_t I2C_0_init();

#ifdef __cplusplus
}
#endif

#endif /* _I2C_LITE_H_INCLUDED */
