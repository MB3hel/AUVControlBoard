
/**
 * \file
 *
 * \brief I2C related functionality implementation.
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

#include "i2c_lite.h"

/**
 * \brief Initialize I2C interface
 */
int8_t I2C_0_init()
{

	if (!hri_sercomi2cm_is_syncing(SERCOM2, SERCOM_I2CM_SYNCBUSY_SWRST)) {
		uint32_t mode = SERCOM_I2CM_CTRLA_MODE(5);
		if (hri_sercomi2cm_get_CTRLA_reg(SERCOM2, SERCOM_I2CM_CTRLA_ENABLE)) {
			hri_sercomi2cm_clear_CTRLA_ENABLE_bit(SERCOM2);
			hri_sercomi2cm_wait_for_sync(SERCOM2, SERCOM_I2CM_SYNCBUSY_ENABLE);
		}
		hri_sercomi2cm_write_CTRLA_reg(SERCOM2, SERCOM_I2CM_CTRLA_SWRST | mode);
	}
	hri_sercomi2cm_wait_for_sync(SERCOM2, SERCOM_I2CM_SYNCBUSY_SWRST);

	hri_sercomi2cm_write_CTRLA_reg(
	    SERCOM2,
	    0 << SERCOM_I2CM_CTRLA_LOWTOUTEN_Pos      /* SCL Low Time-Out: disabled */
	        | 3 << SERCOM_I2CM_CTRLA_INACTOUT_Pos /* Inactive Time-Out: 3 */
	        | 1 << SERCOM_I2CM_CTRLA_SCLSM_Pos    /* SCL Clock Stretch Mode: enabled */
	        | 0 << SERCOM_I2CM_CTRLA_SPEED_Pos    /* Transfer Speed: 0 */
	        | 0 << SERCOM_I2CM_CTRLA_SEXTTOEN_Pos /* Slave SCL Low Extend Time-Out: disabled */
	        | 0 << SERCOM_I2CM_CTRLA_MEXTTOEN_Pos /* Master SCL Low Extend Time-Out: 0 */
	        | 0 << SERCOM_I2CM_CTRLA_SDAHOLD_Pos  /* SDA Hold Time: 0 */
	        | 0 << SERCOM_I2CM_CTRLA_PINOUT_Pos   /* Pin Usage: disabled */
	        | 0 << SERCOM_I2CM_CTRLA_RUNSTDBY_Pos /* Run In Standby: disabled */
	        | 5 << SERCOM_I2CM_CTRLA_MODE_Pos);   /* Operating Mode: 5 */

	// hri_sercomi2cm_write_CTRLB_reg(SERCOM2,0 << SERCOM_I2CM_CTRLB_CMD_Pos /* Command: 0 */
	//		 | 0 << SERCOM_I2CM_CTRLB_QCEN_Pos /* Quick Command Enable: disabled */
	//		 | 0 << SERCOM_I2CM_CTRLB_ACKACT_Pos /* Acknowledge Action: disabled */
	//		 | 0 << SERCOM_I2CM_CTRLB_SMEN_Pos); /* Smart Mode Enable: disabled */

	hri_sercomi2cm_write_BAUD_reg(SERCOM2, SERCOM2_BAUD_RATE);

	// hri_sercomi2cm_write_DBGCTRL_reg(SERCOM2,0 << SERCOM_I2CM_DBGCTRL_DBGSTOP_Pos); /* Debug Stop Mode: disabled */

	hri_sercomi2cm_write_INTEN_reg(
	    SERCOM2,
	    1 << SERCOM_I2CM_INTENSET_ERROR_Pos      /* Error Interrupt Enable: enabled */
	        | 1 << SERCOM_I2CM_INTENSET_SB_Pos   /* Slave on Bus Interrupt Enable: enabled */
	        | 1 << SERCOM_I2CM_INTENSET_MB_Pos); /* Master on Bus Interrupt Enable: enabled */

	// hri_sercomi2cm_write_CTRLA_ENABLE_bit(SERCOM2,0 << SERCOM_I2CM_CTRLA_ENABLE_Pos); /* Enable: disabled */

	return 0;
}
