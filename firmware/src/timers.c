/**
 * @file timers.c
 * @author Marcus Behel
 */

#include <timers.h>
#include <clocks.h>
#include <flags.h>
#include <sam.h>
#include <util.h>
#include <dotstar.h>


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
    TC0->COUNT16.CTRLA.bit.ENABLE = 0;                              // Disable TC0
    while(TC0->COUNT16.SYNCBUSY.bit.ENABLE);                        // Wait for sync
    MCLK->APBAMASK.bit.TC0_ = 1;                                    // Enable APB clock to TC0
    TC0->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;                        // Rest TC0
    while(TC0->COUNT16.SYNCBUSY.bit.SWRST);                         // Wait for reset
    TC0->COUNT16.CTRLA.bit.MODE = 
            TC_CTRLA_MODE_COUNT16_Val;                              // 16-bit mode
    TC0->COUNT16.WAVE.bit.WAVEGEN = 
            TC_WAVE_WAVEGEN_NFRQ_Val;                               // Normal Frequency mode (count resets at max)
    TC0->COUNT16.CTRLA.bit.PRESCALER = 
            TC_CTRLA_PRESCALER_DIV8_Val;                            // 120MHz / 8 / 15MHz
    TC0->COUNT16.CTRLA.bit.PRESCSYNC = 
            TC_CTRLA_PRESCSYNC_GCLK;                                // Use GCLK presync method
    TC0->COUNT16.COUNT.reg = 0;                                     // Zero count
    while(TC0->COUNT16.SYNCBUSY.bit.COUNT);                         // Wait for sync
    TC0->COUNT16.CC[0].reg = TC0_CC0_OFFSET;                        // Initial interrupt time
    while(TC0->COUNT16.SYNCBUSY.bit.CC0);                           // Wait for sync
    TC0->COUNT16.CC[1].reg = TC0_CC1_OFFSET;                        // Initial interrupt time
    while(TC0->COUNT16.SYNCBUSY.bit.CC1);                           // Wait for sync
    TC0->COUNT16.INTFLAG.reg |= TC_INTFLAG_MASK;                    // Clear all interrupt flags
    TC0->COUNT16.INTENSET.bit.MC0 = 1;                              // Enable match channel 0 interrupt
    NVIC_EnableIRQ(TC0_IRQn);                                       // Enable TC0 Interrupt handler
    TC0->COUNT16.CTRLA.bit.ENABLE = 1;                              // Enable TC0
    while(TC0->COUNT16.SYNCBUSY.bit.ENABLE);                        // Wait for sync
}

/**
 * TC1: Unused
 */
void timers_tc1_init(void){
    
}

/**
 * TC2: Unused
 */
void timers_tc2_init(void){
    
}

/**
 * TC3: Unused
 */
void timers_tc3_init(void){
    
}

/**
 * TCC0: Used for thruster PWM signals
 * Thruster 1: DIO 13 = PA22 = TCC0[2]
 * Thruster 2: DIO 12 = PA23 = TCC0[3]
 * Thruster 3: DIO 11 = PA21 = TCC0[1]
 * Thruster 4: DIO 10 = PA20 = TCC0[0]
 */
void timers_tcc0_init(void){
    TCC0->CTRLA.bit.ENABLE = 0;                                     // Disable TCC0
    while(TCC0->SYNCBUSY.bit.ENABLE);                               // Wait for sync
    MCLK->APBBMASK.bit.TCC0_ = 1;                                   // Enable APB clock to TCC0
    TCC0->CTRLA.bit.SWRST = 1;                                      // Reset TCC0
    while(TCC0->SYNCBUSY.bit.SWRST);                                // Wait for reset
    TCC0->CTRLA.bit.PRESCALER = TCC_CTRLA_PRESCALER_DIV16_Val;      // Divide clock by 16 (48MHz / 16 = 3MHz)
    TCC0->CTRLA.bit.PRESCSYNC = TCC_CTRLA_PRESCSYNC_GCLK_Val;       // Sync by GCLK
    TCC0->CTRLA.bit.RESOLUTION = TCC_CTRLA_RESOLUTION_NONE_Val;     // No dithering
    TCC0->CC[0].reg = 0x1194;                                       // Default to 1500us pulse
    while(TCC0->SYNCBUSY.bit.CC0);                                  // Wait for sync
    TCC0->CC[1].reg = 0x1194;                                       // Default to 1500us pulse
    while(TCC0->SYNCBUSY.bit.CC1);                                  // Wait for sync
    TCC0->CC[2].reg = 0x1194;                                       // Default to 1500us pulse
    while(TCC0->SYNCBUSY.bit.CC2);                                  // Wait for sync
    TCC0->CC[3].reg = 0x1194;                                       // Default to 1500us pulse
    while(TCC0->SYNCBUSY.bit.CC3);                                  // Wait for sync
    TCC0->WAVE.bit.WAVEGEN = TCC_WAVE_WAVEGEN_NPWM_Val;             // Normal PWM wavegen mode
    TCC0->PER.reg = 0x1653;                                         // Waveform period is 1905us
    TCC0->CTRLA.bit.ENABLE = 1;                                     // Enable TCC0
    while(TCC0->SYNCBUSY.bit.ENABLE);                               // Wait for sync
}

/**
 * TCC1: Used for thruster PWM signals
 * Thruster 5: DIO  9 = PA19 = TCC1[3]
 * Thruster 6: DIO  7 = PA18 = TCC1[2]
 * Thruster 7: DIO  1 = PA17 = TCC1[1]
 * Thruster 8: DIO  0 = PA16 = TCC1[0]
 */
void timers_tcc1_init(void){
    TCC1->CTRLA.bit.ENABLE = 0;                                     // Disable TCC1
    while(TCC1->SYNCBUSY.bit.ENABLE);                               // Wait for sync
    MCLK->APBBMASK.bit.TCC1_ = 1;                                   // Enable APB clock to TCC1
    TCC1->CTRLA.bit.SWRST = 1;                                      // Reset TCC1
    while(TCC1->SYNCBUSY.bit.SWRST);                                // Wait for reset
    TCC1->CTRLA.bit.PRESCALER = TCC_CTRLA_PRESCALER_DIV16_Val;      // Divide clock by 16 (48MHz / 16 = 3MHz)
    TCC1->CTRLA.bit.PRESCSYNC = TCC_CTRLA_PRESCSYNC_GCLK_Val;       // Sync by GCLK
    TCC1->CTRLA.bit.RESOLUTION = TCC_CTRLA_RESOLUTION_NONE_Val;     // No dithering
    TCC1->CC[0].reg = 0x1194;                                       // Default to 1500us pulse
    while(TCC1->SYNCBUSY.bit.CC0);                                  // Wait for sync
    TCC1->CC[1].reg = 0x1194;                                       // Default to 1500us pulse
    while(TCC1->SYNCBUSY.bit.CC1);                                  // Wait for sync
    TCC1->CC[2].reg = 0x1194;                                       // Default to 1500us pulse
    while(TCC1->SYNCBUSY.bit.CC2);                                  // Wait for sync
    TCC1->CC[3].reg = 0x1194;                                       // Default to 1500us pulse
    while(TCC1->SYNCBUSY.bit.CC3);                                  // Wait for sync
    TCC1->WAVE.bit.WAVEGEN = TCC_WAVE_WAVEGEN_NPWM_Val;             // Normal PWM wavegen mode
    TCC1->PER.reg = 0x1653;                                         // Waveform period is 1905us
    TCC1->CTRLA.bit.ENABLE = 1;                                     // Enable TCC1
    while(TCC1->SYNCBUSY.bit.ENABLE);                               // Wait for sync
}

/**
 * TCC2: Unused
 */
void timers_tcc2_init(void){

}

/**
 * Initialize WDT
 * Note: Counts at fixed 1024Hz rate on this chip
 */
void timers_wdt_init(void){
    MCLK->APBAMASK.bit.WDT_ = 1;                                    // Enable APB clock to WDT

    WDT->CTRLA.bit.ENABLE = 0;                                      // Disable before config
    while(WDT->SYNCBUSY.bit.ENABLE);                                // Wait for sync
    WDT->CTRLA.bit.WEN = 0;                                         // Disable window mode
    while(WDT->SYNCBUSY.bit.WEN);                                   // Wait for sync
    WDT->CONFIG.bit.PER = WDT_CONFIG_PER_CYC2048_Val;               // 2048 cycles = 2 seconds
    WDT->CTRLA.bit.ENABLE = 1;                                      // Enable WDT
    while(WDT->SYNCBUSY.bit.ENABLE);                                // Wait for sync
}

void timers_init(void){
    // -----------------------------------------------------------------------------------------------------------------
    // GCLK config (shared between some timers)
    // -----------------------------------------------------------------------------------------------------------------
    // See page 156 (table 14-9) in datasheet for which are shared
    // Also note that 32-bit mode PAIRS TWO TCS
    // See page 1549 of datasheet for details

    // TC0 and TC1 shared (120MHz)
    GCLK->PCHCTRL[TC0_GCLK_ID].bit.GEN = CLOCKS_GCLK_120M;          // Select 120MHz GCLK for ref
    GCLK->PCHCTRL[TC0_GCLK_ID].bit.CHEN = 1;                        // Enable channel

    // TC2 and TC3 shared
    // Not used

    // TCC0 and TCC1 shared (48Mhz)
    GCLK->PCHCTRL[TCC0_GCLK_ID].bit.GEN = CLOCKS_GCLK_48M;          // Select 48MHz GCLK for ref
    GCLK->PCHCTRL[TCC0_GCLK_ID].bit.CHEN = 1;                       // Enable channel

    // TCC2 not shared (no TCC3 on this chip)
    // Not used
    // -----------------------------------------------------------------------------------------------------------------

    // -----------------------------------------------------------------------------------------------------------------
    // Init each timer
    // -----------------------------------------------------------------------------------------------------------------
    timers_tc0_init();
    timers_tc1_init();
    timers_tc2_init();
    timers_tc3_init();
    timers_tcc0_init();
    timers_tcc1_init();
    timers_wdt_init();
    // -----------------------------------------------------------------------------------------------------------------
}

void timers_thruster_pwm_set(float *speeds){
    #define TCALC 3                                                 // timer counts / usec (3MHz clock)
    #define PULSE_WDITH(speed)  ((400*speed) + 1500)		        // Convert -1.0 to 1.0 range to pulse high time

    // ESCs use 1500 microsecond pulse for 0
    // 1100 microsecond pulse for -1
    // and 1900 microsecond pulse for 1

    // Thruster 1: TCC0 CC2
    TCC0->CC[2].reg = PULSE_WDITH(speeds[0]) * TCALC;
    while(TCC0->SYNCBUSY.bit.CC2);
    
    // Thruster 2: TCC0 CC3
    TCC0->CC[3].reg = PULSE_WDITH(speeds[1]) * TCALC;
    while(TCC0->SYNCBUSY.bit.CC3);

    // Thruster 3: TCC0 CC1
    TCC0->CC[1].reg = PULSE_WDITH(speeds[2]) * TCALC;
    while(TCC0->SYNCBUSY.bit.CC1);

    // Thruster 4: TCC0 CC0
    TCC0->CC[0].reg = PULSE_WDITH(speeds[3]) * TCALC;
    while(TCC0->SYNCBUSY.bit.CC0);

    // Thruster 5: TCC1 CC3
    TCC1->CC[3].reg = PULSE_WDITH(speeds[4]) * TCALC;
    while(TCC1->SYNCBUSY.bit.CC3);

    // Thruster 6: TCC1 CC2
    TCC1->CC[2].reg = PULSE_WDITH(speeds[5]) * TCALC;
    while(TCC1->SYNCBUSY.bit.CC2);

    // Thruster 7: TCC1 CC1
    TCC1->CC[1].reg = PULSE_WDITH(speeds[6]) * TCALC;
    while(TCC1->SYNCBUSY.bit.CC1);

    // Thruster 8: TCC1 CC0
    TCC1->CC[0].reg = PULSE_WDITH(speeds[7]) * TCALC;
    while(TCC1->SYNCBUSY.bit.CC0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// IRQ Handlers
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TC0_Handler(void){ 
    static unsigned int ctr10ms = 10;                               // Count down from 10ms
    static unsigned int ctr20ms = 20;                               // Count down from 20ms
    static unsigned int ctr50ms = 50;                               // Count down from 50ms
    static unsigned int ctr100ms = 100;                             // Count down from 100ms
    static unsigned int ctr1000ms = 1000;                           // Count down from 1000ms

    if(TC0->COUNT16.INTFLAG.bit.MC0){
        // CC0 matched (1ms interrupt rate)

        // Decrement counters
        ctr10ms--;
        ctr20ms--;
        ctr50ms--;
        ctr100ms--;
        ctr1000ms--;

        // Handle counters at zero
        if(ctr10ms == 0){
            FLAG_SET(flags_main, FLAG_MAIN_10MS);
            ctr10ms = 10;
        }
        if(ctr20ms == 0){
            FLAG_SET(flags_main, FLAG_MAIN_20MS);
            ctr20ms = 20;
        }
        if(ctr50ms == 0){
            FLAG_SET(flags_main, FLAG_MAIN_50MS);
            ctr50ms = 50;
        }
        if(ctr100ms == 0){
            FLAG_SET(flags_main, FLAG_MAIN_100MS);
            ctr100ms = 100;
        }
        if(ctr1000ms == 0){
            FLAG_SET(flags_main, FLAG_MAIN_1000MS);
            ctr1000ms = 1000;
        }
        

        // Configure to  interrupt again at configured period
        TC0->COUNT16.CC[0].reg += TC0_CC0_OFFSET;                    // Adjust by offset
        // NOTE: No need to wait for sync here and delay IRQ handler
        //       This won't be accessed again until next IRQ handler
        //       At which point it must have synchronized
        TC0->COUNT16.INTFLAG.reg |= TC_INTFLAG_MC0;                  // Clear flag
    }
    if(TC0->COUNT16.INTFLAG.bit.MC1){
        // CC1 matched (10us interrupt rate)
        // TODO: Implement things here later

        // Configure to  interrupt again at configured period
        TC0->COUNT16.CC[1].reg += TC0_CC1_OFFSET;                    // Adjust by offset
        // NOTE: No need to wait for sync here and delay IRQ handler
        //       This won't be accessed again until next IRQ handler
        //       At which point it must have synchronized
        TC0->COUNT16.INTFLAG.reg |= TC_INTFLAG_MC1;                  // Clear flag
    }
}
