/**
 * Program entry point, main tree, and ISRs
 * @file main.c
 * @author Marcus Behel (mgbehel@ncsu.edu)
 */

#include <atmel_start.h>
#include <pccomm.h>
#include <motor_pwm.h>


#define P 0 // Port PA
#define T1 22
#define T2 23
#define T3 21
#define T4 20
#define T5 19
#define T6 18
#define T7 17
#define T8 16

/*int main(void){
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Initialization
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    atmel_start_init();                             // Initialize ASF4 drivers & middleware
    pccomm_init();                                  // Initialize USB communications with PC
    motor_pwm_init();                               // Initialize motor PWM subsystem
    

    float speeds[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    motor_pwm_set(speeds);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Main loop
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    while (1) {
        
    }
}*/


int main(void){
    atmel_start_init();

    GCLK->PCHCTRL[25].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN(1); // Uses 48 MHz clock
	while(GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL_GCLK1);

    MCLK->APBBMASK.reg |= MCLK_APBBMASK_TCC1;	
    TCC1->CTRLA.reg = TCC_CTRLA_SWRST; 
    while(TCC1->SYNCBUSY.reg & TCC_SYNCBUSY_SWRST);

    TCC1->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV4; // divides frequency by 4
    TCC1->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;

    TCC1->PER.reg = 5700;

    PORT->Group[P].PINCFG[T1].reg = PORT_PINCFG_PMUXEN;
	PORT->Group[P].PMUX[T1/2].reg = PORT_PMUX_PMUXE(5);

    TCC1->CC[0].reg = 4500;
}
