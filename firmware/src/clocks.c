/**
 * @file clocks.c
 * @author Marcus Behel
 */

#include <clocks.h>
#include <sam.h>
#include <ports.h>


void clocks_init(void){
    // TODO: Why???
    NVMCTRL->CTRLA.bit.RWS = 0;

    GCLK->CTRLA.bit.SWRST = 1;                                          // Reset GCLKs
    while( GCLK->SYNCBUSY.bit.SWRST);                                   // Wait for reset

    // GCLK3 -> 32.768kHz from internal oscillator
    GCLK->GENCTRL[3].bit.SRC = GCLK_GENCTRL_SRC_OSCULP32K_Val;          // GCLK 3 Sourced from internal 32k
    GCLK->GENCTRL[3].bit.GENEN = 1;                                     // Enable GCLK 3
    while(GCLK->SYNCBUSY.bit.GENCTRL3);                                 // Wait for sync

    // OSCCTRL setup
    MCLK->APBAMASK.bit.OSCCTRL_ = 1;                                    // Enable APB clock to OSCCTRL

    // DFLL48M Config (reference clock = 32.768kHz)
    GCLK->PCHCTRL[OSCCTRL_GCLK_ID_DFLL48].bit.GEN = 3;                  // Sourced from GCLK3
    GCLK->PCHCTRL[OSCCTRL_GCLK_ID_DFLL48].bit.CHEN = 1;                 // Enable clock to DFLL48
    OSCCTRL->DFLLMUL.bit.CSTEP = 1;                                     // Coarse max step
    OSCCTRL->DFLLMUL.bit.FSTEP = 1;                                     // Fine max step
    OSCCTRL->DFLLMUL.bit.MUL = 0;                                       // Multiply factor
    while(OSCCTRL->DFLLSYNC.bit.DFLLMUL);                               // Wait for sync
    OSCCTRL->DFLLCTRLB.bit.MODE = 0;                                    // Open loop mode
    OSCCTRL->DFLLCTRLB.bit.WAITLOCK = 1;                                // Wait for lock before output
    while(OSCCTRL->DFLLSYNC.bit.DFLLCTRLB);                             // Wait for sync
    OSCCTRL->DFLLCTRLA.bit.RUNSTDBY = 1;                                // Enable run in standby mode
    OSCCTRL->DFLLCTRLA.bit.ENABLE = 1;                                  // Enable DFLL48M
    while(OSCCTRL->DFLLSYNC.bit.ENABLE);                                // Wait for sync

    // GCLK1 -> 48MHz from DFLL48M
    GCLK->GENCTRL[1].bit.SRC = GCLK_GENCTRL_SRC_DFLL_Val;               // GCLK1 sourced from DFLL
    GCLK->GENCTRL[1].bit.GENEN = 1;                                     // Enable GCLK1
    while(GCLK->SYNCBUSY.bit.GENCTRL1);                                 // Wait for sync

    // DPLL0 Config (reference clock = 32.768kHz)
    GCLK->PCHCTRL[OSCCTRL_GCLK_ID_FDPLL0].reg = GCLK_PCHCTRL_CHEN       // Source from GCLK 3 and enable channel
            | GCLK_PCHCTRL_GEN(3);
    OSCCTRL->Dpll[0].DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDRFRAC(0x00) |  // Set ratio
            OSCCTRL_DPLLRATIO_LDR((F_CPU - 500000) / 1000000);
    
    while(OSCCTRL->Dpll[0].DPLLSYNCBUSY.bit.DPLLRATIO);
    
    //MUST USE LBYPASS DUE TO BUG IN REV A OF SAMD51
    OSCCTRL->Dpll[0].DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_REFCLK_GCLK | OSCCTRL_DPLLCTRLB_LBYPASS;
    
    OSCCTRL->Dpll[0].DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE;
    
    while( OSCCTRL->Dpll[0].DPLLSTATUS.bit.CLKRDY == 0 || OSCCTRL->Dpll[0].DPLLSTATUS.bit.LOCK == 0 );    

    ports_gpio_set(P_RED_LED);

    // GCLK0 -> 120MHz from DPLL0
    // GCLK0 is always the source for MCLK
    GCLK->GENCTRL[0].bit.SRC = GCLK_GENCTRL_SRC_DPLL0_Val;              // GCLK0 soruced from DPLL0
    GCLK->GENCTRL[0].bit.IDC = 1;                                       // Set IDC bit
    GCLK->GENCTRL[0].bit.GENEN = 1;                                     // Enable GCLK0
    while(GCLK->SYNCBUSY.bit.GENCTRL0);                                 // Wait for sync
    MCLK->CPUDIV.reg = MCLK_CPUDIV_DIV_DIV1;                            // Configure MCLK divider for CPU clock

    
}
