#pragma once

// Note: Names delay_init, delay_us and delay_ms are defined by ASF4 used in ControlBoard v1, so using different names

void delay_initialize(void);

void delay_micros(unsigned int us);

void delay_millis(unsigned int ms);

void delay_seconds(unsigned int sec);
