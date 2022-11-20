#pragma once

#if defined(CONTROL_BOARD_V1)

#define init_frameworks()   SYS_Initialize(NULL)

#include <samd51g19a.h>
#include <definitions.h>
#include <device.h>

#elif defined(CONTROL_BOARD_V2)

#define init_frameworks     stm32cubemx_main

#include <stm32f4xx.h>
#include <stm32f4xx_hal.h>
#include <stm32cubemx_main.h>

extern void stm32cubemx_main(void);

#endif