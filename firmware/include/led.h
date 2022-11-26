#pragma once

#include <stdint.h>

void led_init(void);

void led_on(void);

void led_off(void);

void led_toggle(void);

void led_rgb_set(uint8_t r, uint8_t g, uint8_t b);