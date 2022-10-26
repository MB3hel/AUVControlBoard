/**
 * @file timers.c
 * @author Marcus Behel
 */

#include <timers.h>
#include <clocks.h>
#include <sam.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define TC0_CC0_OFFSET          15000                               // 15MHz / 15000 = 1000Hz IRQ rate (1ms period)
#define TC0_CC1_OFFSET          150                                 // 15MHz / 150 = 100kHz IRQ rate (10us period)


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * TC0: General timing
 * CC0 Generates interrupt every 1ms
 * CC1 Generates interrupt every 10us
 */
void timers_tc0_init(void){
    // Enable APB clock to TC0
    MCLK->APBAMASK.bit.TC0_ = 1;
    
    TC0->COUNT32.CTRLA.bit.SWRST = TC_CTRLA_SWRST;                  // Reset TC0 regs (see datasheet)
    while(TC0->COUNT32.SYNCBUSY.bit.SWRST);                         // Wait for sync
    TC0->COUNT32.CTRLA.bit.CAPTMODE0 = 
        TC_CTRLA_CAPTMODE0_DEFAULT_Val;                             // Default capture mode (Ch0)
    TC0->COUNT32.CTRLA.bit.CAPTMODE1 = 
        TC_CTRLA_CAPTMODE0_DEFAULT_Val;                             // Default capture mode (Ch1)
    TC0->COUNT32.CTRLA.bit.CAPTEN0 = 0;                             // Disable capture (Ch0)
    TC0->COUNT32.CTRLA.bit.CAPTEN1 = 0;                             // Disable capture (Ch1)
    TC0->COUNT32.CTRLA.bit.PRESCALER = TC_CTRLA_PRESCALER_DIV8;     // 120MHz / 8 = 15MHz count rate
    TC0->COUNT32.CTRLA.bit.RUNSTDBY = 1;                            // Run in standby mode
    TC0->COUNT32.CTRLA.bit.MODE = TC_CTRLA_MODE_COUNT32_Val;        // Put timer in 32-bit mode
    
    TC0->COUNT32.CTRLBCLR.bit.DIR = 1;                              // Clear count direction bit (set to 0) = count up
    while(TC0->COUNT32.SYNCBUSY.bit.CTRLB);                         // Wait for sync

    TC0->COUNT32.COUNT.reg = 0;                                     // Start from 0
    while(TC0->COUNT32.SYNCBUSY.bit.COUNT);                         // Wait for sync
    
    TC0->COUNT32.CC[0].reg = TC0_CC0_OFFSET;                        // Set initial CC0 value
    while(TC0->COUNT32.SYNCBUSY.bit.CC0);                           // Wait for sync
    
    TC1->COUNT32.CC[1].reg = TC0_CC1_OFFSET;                        // Set initial CC0 value
    while(TC0->COUNT32.SYNCBUSY.bit.CC1);                           // Wait for sync

    TC0->COUNT32.INTENSET.bit.MC0 = 1;                              // Enable match channel 0 interrupt
    TC0->COUNT32.INTENSET.bit.MC1 = 1;                              // Enable match channel 1 interrupt

    NVIC_EnableIRQ(TC0_IRQn);                                       // Enable TC0 Interrupt handler

    TC0->COUNT32.CTRLA.bit.ENABLE = 1;                              // Enable TC0
    while(TC0->COUNT32.SYNCBUSY.bit.ENABLE);                        // Wait for sync
}

/**
 * TC1: Unused
 */
void timers_tc1_init(void){
    // NOTE: GCLK ref for TC0 and TC1 is shared. Only configure in once place
    // See page 156 (table 14-9) in datasheet
}

/**
 * TC2: Unused
 */
void timers_tc2_init(void){
    // NOTE: GCLK ref for TC2 and TC3 is shared. Only configure in once place
    // See page 156 (table 14-9) in datasheet
}

/**
 * TC3: Unused
 */
void timers_tc3_init(void){
    // NOTE: GCLK ref for TC2 and TC3 is shared. Only configure in once place
    // See page 156 (table 14-9) in datasheet
}

/**
 * TCC0: Used for thruster PWM signals
 * Thruster 1: DIO 13 = PA22 = TCC0[2]
 * Thruster 2: DIO 12 = PA23 = TCC0[3]
 * Thruster 3: DIO 11 = PA21 = TCC0[1]
 * Thruster 4: DIO 10 = PA20 = TCC0[0]
 */
void timers_tcc0_init(void){
    // NOTE: GCLK ref for TCC0 and TCC1 is shared. Only configure in once place
    // See page 156 (table 14-9) in datasheet
}

/**
 * TCC1: Used for thruster PWM signals
 * Thruster 5: DIO  9 = PA19 = TCC1[3]
 * Thruster 6: DIO  7 = PA18 = TCC1[2]
 * Thruster 7: DIO  1 = PA17 = TCC1[1]
 * Thruster 8: DIO  0 = PA16 = TCC1[0]
 */
void timers_tcc1_init(void){
    // NOTE: GCLK ref for TCC0 and TCC1 is shared. Only configure in once place
    // See page 156 (table 14-9) in datasheet
}

/**
 * TCC2: Unused
 */
void timers_tcc2_init(void){

}

void timers_init(void){
    // -----------------------------------------------------------------------------------------------------------------
    // GCLK config (shared between some timers)
    // -----------------------------------------------------------------------------------------------------------------
    // See page 156 (table 14-9) in datasheet for which are shared

    // TC0 and TC1 shared (120MHz)
    GCLK->PCHCTRL[TC0_GCLK_ID].bit.GEN = CLOCKS_GCLK_120M;          // Select 120MHz GCLK for ref
    GCLK->PCHCTRL[TC0_GCLK_ID].bit.CHEN = 1;                        // Enable channel

    // TC2 and TC3 shared
    // Not used

    // TCC0 and TCC1 shared
    // Not used

    // TCC2 not shared (no TCC3 on this chip)
    // Not used
    // -----------------------------------------------------------------------------------------------------------------

    // Init each timer
    timers_tc0_init();
    timers_tc1_init();
    timers_tc2_init();
    timers_tc3_init();
    timers_tcc0_init();
    timers_tcc1_init();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// IRQ Handlers
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TC0_Handler(void){
    if(TC0->COUNT32.INTFLAG.bit.MC0){
        // CC0 matched (1ms interrupt rate)
        // TODO: Is this even useful???
    }
}
