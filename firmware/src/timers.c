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


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * TC0
 * CC0 General timing (1ms interrupt period)
 * CC1 Unused (reserved for i2c0 timeout if needed in future)
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
    TC0->COUNT16.INTENSET.bit.MC0 = 1;                              // Enable match channel 0 interrupt
    TC0->COUNT16.INTFLAG.reg |= TC_INTFLAG_MASK;                    // Clear all interrupt flags
    NVIC_EnableIRQ(TC0_IRQn);                                       // Enable TC0 Interrupt handler
    TC0->COUNT16.CTRLA.bit.ENABLE = 1;                              // Enable TC0
    while(TC0->COUNT16.SYNCBUSY.bit.ENABLE);                        // Wait for sync
}

/**
 * TC1
 * CC0 Used for BNO055 delay
 * CC1 (NYI) Used for depth sensor delay
 */
void timers_tc1_init(void){
    TC1->COUNT16.CTRLA.bit.ENABLE = 0;                              // Disable TC1
    while(TC1->COUNT16.SYNCBUSY.bit.ENABLE);                        // Wait for sync
    MCLK->APBAMASK.bit.TC1_ = 1;                                    // Enable APB clock to TC1
    TC1->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;                        // Rest TC1
    while(TC1->COUNT16.SYNCBUSY.bit.SWRST);                         // Wait for reset
    TC1->COUNT16.CTRLA.bit.MODE = 
            TC_CTRLA_MODE_COUNT16_Val;                              // 16-bit mode
    TC1->COUNT16.WAVE.bit.WAVEGEN = 
            TC_WAVE_WAVEGEN_NFRQ_Val;                               // Normal Frequency mode (count resets at max)
    TC1->COUNT16.CTRLA.bit.PRESCALER = 
            TC_CTRLA_PRESCALER_DIV1024_Val;                         // 120MHz / 1024 = 117.1875kHz
    TC1->COUNT16.CTRLA.bit.PRESCSYNC = 
            TC_CTRLA_PRESCSYNC_GCLK;                                // Use GCLK presync method
    TC1->COUNT16.COUNT.reg = 0;                                     // Zero count
    while(TC1->COUNT16.SYNCBUSY.bit.COUNT);                         // Wait for sync
    TC1->COUNT16.INTENCLR.bit.MC0 = 1;                              // Disable match channel 0 interrupt
    TC1->COUNT16.INTFLAG.reg |= TC_INTFLAG_MASK;                    // Clear all interrupt flags
    NVIC_EnableIRQ(TC1_IRQn);                                       // Enable TC1 Interrupt handler
    TC1->COUNT16.CTRLA.bit.ENABLE = 1;                              // Enable TC1
    while(TC1->COUNT16.SYNCBUSY.bit.ENABLE);                        // Wait for sync
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
    // NOTE THAT A TC in 32-BIT MODE IS ACHEIVED USING TWO TC's
    // SEE PAGE 1549 OF DATASHEET FOR DETAILS

    timers_tc0_init();
    timers_tc1_init();
    timers_tc2_init();
    timers_tc3_init();
    timers_tcc0_init();
    timers_tcc1_init();
    timers_wdt_init();
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

void timers_bno055_delay(uint32_t delay){
    // 117.1875kHz clock therefore 117.1875 counts / ms
    // Max delay of 65535 / 117.1875 = 559ms
    if(delay > 559)
        delay = 559;
    TC1->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_READSYNC_Val;   // Setup to read count
    while(TC1->COUNT16.SYNCBUSY.bit.CTRLB);                         // Wait for sync 
    TC1->COUNT16.CC[0].reg = TC1->COUNT16.COUNT.reg +               // Set interrupt time to delay ms in future
            ((delay * 1171875) / 10000);
    TC1->COUNT16.INTFLAG.reg |= TC_INTFLAG_MC0;                     // Clear interrupt flag
    TC1->COUNT16.INTENSET.bit.MC0 = 1;                              // Enable interrupt
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
        // General timing

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
}

void TC1_Handler(void){
    if(TC1->COUNT16.INTFLAG.bit.MC0){
        // CC0 matched (bno055 delay)
        FLAG_SET(flags_main, FLAG_MAIN_BNO055_DELAY);               // Set flag
        TC1->COUNT16.INTENCLR.bit.MC0 = 1;                          // Disable interrupt
        TC1->COUNT16.INTFLAG.reg |= TC_INTFLAG_MC0;                 // Clear flag
    }
}
