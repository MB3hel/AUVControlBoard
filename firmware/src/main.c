/**
 * Program entry point and main tree
 * @file main.c
 * @author Marcus Behel
 */

#include <clocks.h>
#include <ports.h>
#include <stdint.h>


// Copied from ASF for now. Will rewrite later.
static inline uint32_t _get_cycles_for_us_internal(const uint16_t us, const uint32_t freq, const uint8_t power)
{
	switch (power) {
	case 9:
		return (us * (freq / 1000000) + 2) / 3;
	case 8:
		return (us * (freq / 100000) + 29) / 30;
	case 7:
		return (us * (freq / 10000) + 299) / 300;
	case 6:
		return (us * (freq / 1000) + 2999) / 3000;
	case 5:
		return (us * (freq / 100) + 29999) / 30000;
	case 4:
		return (us * (freq / 10) + 299999) / 300000;
	default:
		return (us * freq + 2999999) / 3000000;
	}
}

/**
 * \brief Retrieve the amount of cycles to delay for the given amount of us
 */
uint32_t _get_cycles_for_us(const uint16_t us)
{
	return _get_cycles_for_us_internal(us, 120000000, 9);
}

/**
 * \brief Retrieve the amount of cycles to delay for the given amount of ms
 */
static inline uint32_t _get_cycles_for_ms_internal(const uint16_t ms, const uint32_t freq, const uint8_t power)
{
	switch (power) {
	case 9:
		return (ms * (freq / 1000000) + 2) / 3 * 1000;
	case 8:
		return (ms * (freq / 100000) + 2) / 3 * 100;
	case 7:
		return (ms * (freq / 10000) + 2) / 3 * 10;
	case 6:
		return (ms * (freq / 1000) + 2) / 3;
	case 5:
		return (ms * (freq / 100) + 29) / 30;
	case 4:
		return (ms * (freq / 10) + 299) / 300;
	default:
		return (ms * (freq / 1) + 2999) / 3000;
	}
}

/**
 * \brief Retrieve the amount of cycles to delay for the given amount of ms
 */
uint32_t _get_cycles_for_ms(const uint16_t ms)
{
	return _get_cycles_for_ms_internal(ms, 120000000, 9);
}
/**
 * \brief Initialize delay functionality
 */
void _delay_init(void *const hw)
{
	(void)hw;
}
/**
 * \brief Delay loop to delay n number of cycles
 */
void _delay_cycles(void *const hw, uint32_t cycles)
{
#ifndef _UNIT_TEST_
	(void)hw;
	(void)cycles;
#if defined(__GNUC__) && (__ARMCOMPILER_VERSION > 6000000) /*  Keil MDK with ARM Compiler 6 */
	__asm(".align 3 \n"
	      "__delay:\n"
	      "subs r1, r1, #1\n"
	      "bhi __delay\n");
#elif defined __GNUC__
	__asm(".syntax unified\n"
	      ".align 3 \n"
	      "__delay:\n"
	      "subs r1, r1, #1\n"
	      "bhi __delay\n"
	      ".syntax divided");
#elif defined __CC_ARM
	__asm("__delay:\n"
	      "subs cycles, cycles, #1\n"
	      "bhi __delay\n");
#elif defined __ICCARM__
	__asm("__delay:\n"
	      "subs r1, r1, #1\n"
	      "bhi.n __delay\n");
#endif
#endif
}


int main(void){
    ports_pinfunc(P_RED_LED, PORT_PINFUNC_GPIO);
    ports_gpio_dir(P_RED_LED, PORT_GPIO_OUT);
    ports_gpio_clear(P_RED_LED);
    clocks_init();
    ports_gpio_set(P_RED_LED);
    while(1){
        ports_gpio_toggle(P_RED_LED);
        _delay_cycles(0, _get_cycles_for_ms(1000));
    }
}

