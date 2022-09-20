/**
 * @file motor_pwm.c
 * @author Marcus Behel (mgbehel@ncsu.edu)
 */

#include <motor_pwm.h>
#include <sam.h>
#include <atmel_start.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define PWM_PERIOD 5700 //1900us * 1/3us based on clock math
#define TCALC 3 // Time in milliseconds it takes to count 1 tick is 1/3
#define P 0 // Port PA
#define T1 22
#define T2 23
#define T3 21
#define T4 20
#define T5 19
#define T6 18
#define T7 17
#define T8 16


#define PULSE_WDITH(speed)  ((400*speed) + 1500)


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void motor_pwm_init(void){
	motor_pwm_set((float[]){0, 0, 0, 0, 0, 0, 0, 0});
}


// Takes in array of speeds;
void motor_pwm_set(float *speeds){
	float pulse1 = PULSE_WDITH(speeds[0]); // pulse width for thruster 1
	TCC0->CC[2].reg = pulse1*TCALC; // Setting PWM duty cycle for thruster 1
	
	float pulse2 = PULSE_WDITH(speeds[1]); // pulse width for thruster 2
	TCC0->CC[3].reg = pulse2*TCALC; // Setting PWM duty cycle for thruster 2
	
	float pulse3 = PULSE_WDITH(speeds[2]); // pulse width for thruster 3
	TCC0->CC[1].reg = pulse3*TCALC; // Setting PWM duty cycle for thruster 3
	
	float pulse4 = PULSE_WDITH(speeds[3]); // pulse width for thruster 4
	TCC0->CC[0].reg = pulse4*TCALC; // Setting PWM duty cycle for thruster 4
	
	float pulse5 = PULSE_WDITH(speeds[4]); // pulse width for thruster 5
	TCC1->CC[3].reg = pulse5*TCALC; // Setting PWM duty cycle for thruster 5
	
	float pulse6 = PULSE_WDITH(speeds[5]); // pulse width for thruster 6
	TCC1->CC[2].reg = pulse6*TCALC; // Setting PWM duty cycle for thruster 6
	
	float pulse7 = PULSE_WDITH(speeds[6]); // pulse width for thruster 7
	TCC0->CC[3].reg = pulse7*TCALC; // Setting PWM duty cycle for thruster 7

	float pulse8 = PULSE_WDITH(speeds[7]); // pulse width for thruster 8
	TCC0->CC[1].reg = pulse8*TCALC; // Setting PWM duty cycle for thruster 8
}