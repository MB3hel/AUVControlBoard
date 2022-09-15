/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAMD51 has 14 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3
#define GPIO_PIN_FUNCTION_E 4
#define GPIO_PIN_FUNCTION_F 5
#define GPIO_PIN_FUNCTION_G 6
#define GPIO_PIN_FUNCTION_H 7
#define GPIO_PIN_FUNCTION_I 8
#define GPIO_PIN_FUNCTION_J 9
#define GPIO_PIN_FUNCTION_K 10
#define GPIO_PIN_FUNCTION_L 11
#define GPIO_PIN_FUNCTION_M 12
#define GPIO_PIN_FUNCTION_N 13

#define SENS_SCL GPIO(GPIO_PORTA, 12)
#define SENS_SDA GPIO(GPIO_PORTA, 13)
#define THR8 GPIO(GPIO_PORTA, 16)
#define THR7 GPIO(GPIO_PORTA, 17)
#define THR6 GPIO(GPIO_PORTA, 18)
#define THR5 GPIO(GPIO_PORTA, 19)
#define THR4 GPIO(GPIO_PORTA, 20)
#define THR3 GPIO(GPIO_PORTA, 21)
#define THR2 GPIO(GPIO_PORTA, 22)
#define THR1 GPIO(GPIO_PORTA, 23)
#define DS_MISO GPIO(GPIO_PORTA, 24)
#define DS_MOSI GPIO(GPIO_PORTB, 2)
#define DS_SCK GPIO(GPIO_PORTB, 3)

#endif // ATMEL_START_PINS_H_INCLUDED
