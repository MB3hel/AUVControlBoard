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
	gpio_set_pin_direction(PB22, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PB22,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PB22, PINMUX_PB22D_SERCOM5_PAD2);
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

void USB_DEVICE_INSTANCE_PORT_init(void)
{

	gpio_set_pin_direction(DS_MISO,
	                       // <y> Pin direction
	                       // <id> pad_direction
	                       // <GPIO_DIRECTION_OFF"> Off
	                       // <GPIO_DIRECTION_IN"> In
	                       // <GPIO_DIRECTION_OUT"> Out
	                       GPIO_DIRECTION_OUT);

	gpio_set_pin_level(DS_MISO,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	gpio_set_pin_pull_mode(DS_MISO,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(DS_MISO,
	                      // <y> Pin function
	                      // <id> pad_function
	                      // <i> Auto : use driver pinmux if signal is imported by driver, else turn off function
	                      // <PINMUX_PA24H_USB_DM"> Auto
	                      // <GPIO_PIN_FUNCTION_OFF"> Off
	                      // <GPIO_PIN_FUNCTION_A"> A
	                      // <GPIO_PIN_FUNCTION_B"> B
	                      // <GPIO_PIN_FUNCTION_C"> C
	                      // <GPIO_PIN_FUNCTION_D"> D
	                      // <GPIO_PIN_FUNCTION_E"> E
	                      // <GPIO_PIN_FUNCTION_F"> F
	                      // <GPIO_PIN_FUNCTION_G"> G
	                      // <GPIO_PIN_FUNCTION_H"> H
	                      // <GPIO_PIN_FUNCTION_I"> I
	                      // <GPIO_PIN_FUNCTION_J"> J
	                      // <GPIO_PIN_FUNCTION_K"> K
	                      // <GPIO_PIN_FUNCTION_L"> L
	                      // <GPIO_PIN_FUNCTION_M"> M
	                      // <GPIO_PIN_FUNCTION_N"> N
	                      PINMUX_PA24H_USB_DM);

	gpio_set_pin_direction(PA25,
	                       // <y> Pin direction
	                       // <id> pad_direction
	                       // <GPIO_DIRECTION_OFF"> Off
	                       // <GPIO_DIRECTION_IN"> In
	                       // <GPIO_DIRECTION_OUT"> Out
	                       GPIO_DIRECTION_OUT);

	gpio_set_pin_level(PA25,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	gpio_set_pin_pull_mode(PA25,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA25,
	                      // <y> Pin function
	                      // <id> pad_function
	                      // <i> Auto : use driver pinmux if signal is imported by driver, else turn off function
	                      // <PINMUX_PA25H_USB_DP"> Auto
	                      // <GPIO_PIN_FUNCTION_OFF"> Off
	                      // <GPIO_PIN_FUNCTION_A"> A
	                      // <GPIO_PIN_FUNCTION_B"> B
	                      // <GPIO_PIN_FUNCTION_C"> C
	                      // <GPIO_PIN_FUNCTION_D"> D
	                      // <GPIO_PIN_FUNCTION_E"> E
	                      // <GPIO_PIN_FUNCTION_F"> F
	                      // <GPIO_PIN_FUNCTION_G"> G
	                      // <GPIO_PIN_FUNCTION_H"> H
	                      // <GPIO_PIN_FUNCTION_I"> I
	                      // <GPIO_PIN_FUNCTION_J"> J
	                      // <GPIO_PIN_FUNCTION_K"> K
	                      // <GPIO_PIN_FUNCTION_L"> L
	                      // <GPIO_PIN_FUNCTION_M"> M
	                      // <GPIO_PIN_FUNCTION_N"> N
	                      PINMUX_PA25H_USB_DP);
}

/* The USB module requires a GCLK_USB of 48 MHz ~ 0.25% clock
 * for low speed and full speed operation. */
#if (CONF_GCLK_USB_FREQUENCY > (48000000 + 48000000 / 400)) || (CONF_GCLK_USB_FREQUENCY < (48000000 - 48000000 / 400))
#warning USB clock should be 48MHz ~ 0.25% clock, check your configuration!
#endif

void USB_DEVICE_INSTANCE_CLOCK_init(void)
{

	hri_gclk_write_PCHCTRL_reg(GCLK, USB_GCLK_ID, CONF_GCLK_USB_SRC | GCLK_PCHCTRL_CHEN);
	hri_mclk_set_AHBMASK_USB_bit(MCLK);
	hri_mclk_set_APBBMASK_USB_bit(MCLK);
}

void USB_DEVICE_INSTANCE_init(void)
{
	USB_DEVICE_INSTANCE_CLOCK_init();
	usb_d_init();
	USB_DEVICE_INSTANCE_PORT_init();
}

void system_init(void)
{
	init_mcu();

	CRC_0_init();

	I2C_0_init();

	SPI_0_init();

	USB_DEVICE_INSTANCE_init();
}
