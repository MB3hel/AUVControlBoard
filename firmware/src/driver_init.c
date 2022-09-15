/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>

struct crc_sync_descriptor   CRC_0;
struct spi_m_sync_descriptor SPI_0;

struct i2c_m_async_desc I2C_0;

struct pwm_descriptor PWM_0;

/**
 * \brief CRC initialization function
 *
 * Enables CRC peripheral, clocks and initializes CRC driver
 */
void CRC_0_init(void)
{
	hri_mclk_set_APBBMASK_DSU_bit(MCLK);
	crc_sync_init(&CRC_0, DSU);
}

void I2C_0_PORT_init(void)
{

	gpio_set_pin_pull_mode(SENS_SDA,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(SENS_SDA, PINMUX_PA13D_SERCOM4_PAD0);

	gpio_set_pin_pull_mode(SENS_SCL,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(SENS_SCL, PINMUX_PA12D_SERCOM4_PAD1);
}

void I2C_0_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM4_GCLK_ID_CORE, CONF_GCLK_SERCOM4_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM4_GCLK_ID_SLOW, CONF_GCLK_SERCOM4_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	hri_mclk_set_APBDMASK_SERCOM4_bit(MCLK);
}

void I2C_0_init(void)
{
	I2C_0_CLOCK_init();
	i2c_m_async_init(&I2C_0, SERCOM4);
	I2C_0_PORT_init();
}

void SPI_0_PORT_init(void)
{

	gpio_set_pin_level(DS_MOSI,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(DS_MOSI, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(DS_MOSI, PINMUX_PB02D_SERCOM5_PAD0);

	gpio_set_pin_level(DS_SCK,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(DS_SCK, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(DS_SCK, PINMUX_PB03D_SERCOM5_PAD1);

	// Set pin direction to input
	gpio_set_pin_direction(DS_MISO, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(DS_MISO,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(DS_MISO, PINMUX_PA24D_SERCOM5_PAD2);
}

void SPI_0_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM5_GCLK_ID_CORE, CONF_GCLK_SERCOM5_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM5_GCLK_ID_SLOW, CONF_GCLK_SERCOM5_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	hri_mclk_set_APBDMASK_SERCOM5_bit(MCLK);
}

void SPI_0_init(void)
{
	SPI_0_CLOCK_init();
	spi_m_sync_init(&SPI_0, SERCOM5);
	SPI_0_PORT_init();
}

void PWM_0_PORT_init(void)
{

	gpio_set_pin_function(THR8, PINMUX_PA16F_TCC1_WO0);

	gpio_set_pin_function(THR7, PINMUX_PA17F_TCC1_WO1);

	gpio_set_pin_function(THR6, PINMUX_PA18F_TCC1_WO2);

	gpio_set_pin_function(THR5, PINMUX_PA19F_TCC1_WO3);

	gpio_set_pin_function(THR4, PINMUX_PA20F_TCC1_WO4);

	gpio_set_pin_function(THR3, PINMUX_PA21F_TCC1_WO5);

	gpio_set_pin_function(THR2, PINMUX_PA22F_TCC1_WO6);

	gpio_set_pin_function(THR1, PINMUX_PA23F_TCC1_WO7);
}

void PWM_0_CLOCK_init(void)
{

	hri_mclk_set_APBBMASK_TCC1_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, TCC1_GCLK_ID, CONF_GCLK_TCC1_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
}

void PWM_0_init(void)
{
	PWM_0_CLOCK_init();
	PWM_0_PORT_init();
	pwm_init(&PWM_0, TCC1, _tcc_get_pwm());
}

void system_init(void)
{
	init_mcu();

	CRC_0_init();

	I2C_0_init();

	SPI_0_init();

	PWM_0_init();
}
