/**
 * @file motor_pwm.c
 * @author Marcus Behel
 * @author Chima Nwosu
 */

#include <motor_pwm.h>
#include <sam.h>
#include <atmel_start.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define TCALC 3                                         // timer counts / usec
#define PULSE_WDITH(speed)  ((400*speed) + 1500)		// Convert pulse high time in usec to CC value


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void motor_pwm_init(void){
	// Actual initialization  is done by atmel_start_init
	// This just ensures speeds are always initially set to 0
	motor_pwm_set((float[]){0, 0, 0, 0, 0, 0, 0, 0});
}


// Takes in array of speeds;
void motor_pwm_set(float *speeds){
    hri_tcc_write_CC_reg(TCC0, 2, PULSE_WDITH(speeds[0]) * TCALC);          // Thruster 1
    hri_tcc_write_CC_reg(TCC0, 3, PULSE_WDITH(speeds[1]) * TCALC);          // Thruster 2
	hri_tcc_write_CC_reg(TCC0, 1, PULSE_WDITH(speeds[2]) * TCALC);          // Thruster 3
	hri_tcc_write_CC_reg(TCC0, 0, PULSE_WDITH(speeds[3]) * TCALC);          // Thruster 4
    hri_tcc_write_CC_reg(TCC1, 3, PULSE_WDITH(speeds[4]) * TCALC);          // Thruster 5
    hri_tcc_write_CC_reg(TCC1, 2, PULSE_WDITH(speeds[5]) * TCALC);          // Thruster 6
	hri_tcc_write_CC_reg(TCC1, 1, PULSE_WDITH(speeds[6]) * TCALC);          // Thruster 7
	hri_tcc_write_CC_reg(TCC1, 0, PULSE_WDITH(speeds[7]) * TCALC);          // Thruster 8
}