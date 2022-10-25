/**
 * @file clocks.c
 * @author Marcus Behel
 */

#include <clocks.h>
#include <sam.h>


void clocks_init(void){
    // TODO: Figure out why this is needed (or if needed)?
    NVMCTRL->CTRLA.bit.AUTOWS = 1;

    // GCLK3 -> 32.768kHz from internal oscillator
    GCLK->GENCTRL[3].bit.SRC = GCLK_GENCTRL_SRC_OSCULP32K_Val;          // GCLK 3 Sourced from internal 32k
    GCLK->GENCTRL[3].bit.GENEN = 1;                                     // Enable GCLK 3
    while(GCLK->SYNCBUSY.bit.GENCTRL3);                                 // Wait for sync

    // OSCCTRL setup
    MCLK->APBAMASK.bit.OSCCTRL_ = 1;                                    // Enable APB clock to OSCCTRL

    // DFLL48M Config (reference clock = 32.768kHz)
    GCLK->PCHCTRL[OSCCTRL_GCLK_ID_DFLL48].bit.GEN = 3;                  // Sourced from GCLK3
    GCLK->PCHCTRL[OSCCTRL_GCLK_ID_DFLL48].bit.CHEN = 1;                 // Enable clock to DFLL48
    OSCCTRL->DFLLCTRLB.bit.MODE = 0;                                    // Open loop mode
    OSCCTRL->DFLLCTRLB.bit.WAITLOCK = 1;                                // Wait for lock before output
    OSCCTRL->DFLLMUL.bit.CSTEP = 1;                                     // Coarse max step
    OSCCTRL->DFLLMUL.bit.FSTEP = 1;                                     // Fine max step
    OSCCTRL->DFLLMUL.bit.MUL = 0;                                       // Multiply factor
    OSCCTRL->DFLLCTRLA.bit.RUNSTDBY = 1;                                // Enable run in standby mode
    OSCCTRL->DFLLCTRLA.bit.ENABLE = 1;                                  // Enable DFLL48M
    while(OSCCTRL->DFLLSYNC.bit.ENABLE);                                // Wait for sync

    // GCLK1 -> 48MHz from DFLL48M
    GCLK->GENCTRL[1].bit.SRC = GCLK_GENCTRL_SRC_DFLL_Val;               // GCLK1 sourced from DFLL
    GCLK->GENCTRL[1].bit.GENEN = 1;                                     // Enable GCLK1
    while(GCLK->SYNCBUSY.bit.GENCTRL1);                                 // Wait for sync

    // DPLL0 Config (reference clock = 32.768kHz)
    GCLK->PCHCTRL[OSCCTRL_GCLK_ID_FDPLL0].bit.GEN = 3;                  // Sourced from GCLK3
    GCLK->PCHCTRL[OSCCTRL_GCLK_ID_FDPLL0].bit.CHEN = 1;                 // Enable clock to DPLL0
    OSCCTRL->Dpll[0].DPLLCTRLB.bit.REFCLK = 0x00;                       // Ref clock is from GCLKs
    OSCCTRL->Dpll[0].DPLLCTRLB.bit.LBYPASS = 1;                         // Recommended by errata for low freq ref clock
    OSCCTRL->Dpll[0].DPLLCTRLB.bit.LTIME = 0x00;                        // No timeout. Automatic lock.
    OSCCTRL->Dpll[0].DPLLCTRLB.bit.FILTER = 0x00;                       // Proportional integral filter selection
    OSCCTRL->Dpll[0].DPLLCTRLB.bit.DIV = 0x00;                          // Divider selection
    OSCCTRL->Dpll[0].DPLLCTRLB.bit.DCOFILTER = 0x00;                    // DCO filter selection
    OSCCTRL->Dpll[0].DPLLCTRLB.bit.DCOEN = 0;                           // Disable DCO filter
    OSCCTRL->Dpll[0].DPLLRATIO.bit.LDR = 3661;                          // Loop divider integer ratio
    OSCCTRL->Dpll[0].DPLLRATIO.bit.LDRFRAC = 4;                         // Loop divider fractional ratio
    OSCCTRL->Dpll[0].DPLLCTRLA.bit.RUNSTDBY = 1;                        // Enable run in standby
    OSCCTRL->Dpll[0].DPLLCTRLA.bit.ENABLE = 1;                          // Enable DPLL0
    while(OSCCTRL->Dpll->DPLLSYNCBUSY.bit.ENABLE);                      // Wait for sync

    // GCLK0 -> 120MHz from DPLL0
    // GCLK0 is always the source for MCLK
    GCLK->GENCTRL[0].bit.SRC = GCLK_GENCTRL_SRC_DPLL0_Val;              // GCLK0 soruced from DPLL0
    GCLK->GENCTRL[0].bit.GENEN = 1;                                     // Enable GCLK0
    while(GCLK->SYNCBUSY.bit.GENCTRL0);                                 // Wait for sync
}
