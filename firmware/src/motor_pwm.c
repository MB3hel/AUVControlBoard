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
	GCLK->PCHCTRL[25].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN(1); // Uses 48 MHz clock
	while(GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL_GCLK1);
    
    /*Look at this github and these 2 data sheets to figure out how to initialize clock
    https://gist.github.com/nonsintetic/ad13e70f164801325f5f552f84306d6f
    https://ww1.microchip.com/downloads/en/DeviceDoc/SAM_D21_DA1_Family_DataSheet_DS40001882F.pdf (Look at GCLK)
    https://ww1.microchip.com/downloads/aemDocuments/documents/MCU32/ProductDocuments/DataSheets/SAM_D5x_E5x_Family_Data_Sheet_DS60001507G.pdf
    */
    MCLK->APBBMASK.reg |= MCLK_APBBMASK_TCC1;	
    TCC1->CTRLA.reg = TCC_CTRLA_SWRST; 
    while(TCC1->SYNCBUSY.reg & TCC_SYNCBUSY_SWRST);  // Waits for reset to complete
    
    TCC1->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV4; // divides frequency by 4
    TCC1->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;  // Sets waveform to PWM

    MCLK->APBBMASK.reg |= MCLK_APBBMASK_TCC0;
    TCC0->CTRLA.reg = TCC_CTRLA_SWRST; 
    while(TCC0->SYNCBUSY.reg & TCC_SYNCBUSY_SWRST);  // Waits for reset to complete
    TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV4;
    TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM; 

    TCC0->PER.reg = PWM_PERIOD;
    TCC1->PER.reg = PWM_PERIOD;

    // Configure pins as outputs from timer
    
    PORT->Group[P].PINCFG[T1].reg = PORT_PINCFG_PMUXEN;
	PORT->Group[P].PMUX[T1/2].reg = PORT_PMUX_PMUXE(5); // Using function peripheral function F

    PORT->Group[P].PINCFG[T2].reg = PORT_PINCFG_PMUXEN;
	PORT->Group[P].PMUX[T1/2 + 1].reg = PORT_PMUX_PMUXE(5); // Using function peripheral function F

    PORT->Group[P].PINCFG[T3].reg = PORT_PINCFG_PMUXEN;
	PORT->Group[P].PMUX[T3/2 + 1].reg = PORT_PMUX_PMUXE(5); // Using function peripheral function F
	
    PORT->Group[P].PINCFG[T4].reg = PORT_PINCFG_PMUXEN;
	PORT->Group[P].PMUX[T4/2].reg = PORT_PMUX_PMUXE(5); // Using function peripheral function F
	
    PORT->Group[P].PINCFG[T5].reg = PORT_PINCFG_PMUXEN;
	PORT->Group[P].PMUX[T5/2 + 1].reg = PORT_PMUX_PMUXE(5); // Using function peripheral function F
	
    PORT->Group[P].PINCFG[T6].reg = PORT_PINCFG_PMUXEN;
	PORT->Group[P].PMUX[T6/2].reg = PORT_PMUX_PMUXE(5); // Using function peripheral function F

    PORT->Group[P].PINCFG[T7].reg = PORT_PINCFG_PMUXEN;
	PORT->Group[P].PMUX[T7/2 + 1].reg = PORT_PMUX_PMUXE(5); // Using function peripheral function F
	
    PORT->Group[P].PINCFG[T8].reg = PORT_PINCFG_PMUXEN;
	PORT->Group[P].PMUX[T8/2].reg = PORT_PMUX_PMUXE(5); // Using function peripheral function F	

    // Enable timers
    TCC1->CTRLA.reg |= TCC_CTRLA_ENABLE;
    while(TCC1->SYNCBUSY.reg & TCC_SYNCBUSY_ENABLE);

    TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE;
    while(TCC0->SYNCBUSY.reg & TCC_SYNCBUSY_ENABLE);
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