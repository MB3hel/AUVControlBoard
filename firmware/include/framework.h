#pragma once

#if defined(CONTROL_BOARD_V1)

#define init_frameworks     atmel_start_init

#include <atmel_start.h>
#include <sam.h>
#include <peripheral_clk_config.h>

#elif defined(CONTROL_BOARD_V2)

#define init_frameworks     stm32cubemx_main

#include <stm32f4xx.h>
#include <stm32f4xx_hal.h>
#include <stm32cubemx_main.h>

extern void stm32cubemx_main(void);

#endif