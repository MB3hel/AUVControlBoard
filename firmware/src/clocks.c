/**
 * @file clocks.c
 * @author Marcus Behel
 */

#include <clocks.h>
#include <sam.h>
#include <ports.h>
#include <core_cm4.h>

/**
 * Setup for delays using CYCCNT register from M4 arm core
 */
void delay_init(void){
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;             // Set trace enable bit to be able to use DWT
}

void delay_us(uint32_t us){
    uint32_t cycles = us * (SystemCoreClock / 1e6);
    DWT->CYCCNT = 0;                                            // Reset DWT Cycle Counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;                        // Enable cycle counter
    while(DWT->CYCCNT < cycles);
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;                       // Disable cycle counter
}

void delay_ms(uint32_t ms){
    uint32_t cycles = ms * (SystemCoreClock / 1e3);
    DWT->CYCCNT = 0;                                            // Reset DWT Cycle Counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;                        // Enable cycle counter
    while(DWT->CYCCNT < cycles);
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;                       // Disable cycle counter
}

void delay_sec(uint32_t sec){
    // Clock is too fast to do delays longer than 35 seconds
    // CYCCNT is 32-bit counter
    // (2^32-1) / 120000000 = 35.8
    // To make this more generic, just call delay_ms many times
    // Overhead of function calls is small compared to the delay time
    for(uint32_t i = sec; i > 0; --i){
        delay_ms(1000);
    }
}

/**
 * ItsyBitsy M4 Express has not external crystal, thus all clocks must
 * ultimately come from the ultra low power internal 32.768 kHz oscillator
 * (ULP32K)
 * 
 * Required clocks:
 * 120 MHz clock for CPU
 * 48 MHz clock for USB
 * 32.768 kHz clock for "human" timing
 * 
 * See page 136 of datasheet for clock diagram
 * 
 * MCLK generates CPU clock and bus clocks (APB, AHB)
 * GCLK 0 is always the source of MCLK (optional dividers for CPU, APB, AHB)
 * Thus we need
 * GCLK0 = 120 MHz
 * 
 * Choose (arbitrary GCLK numbers here)
 * GCLK1 = 48 MHz
 * GCLK3 = 32.768 kHz
 * 
 * Note: GCLK2 used to be used for 1MHz (DFLL / 48), but not needed so removed
 * 
 * The internal DFLL can generate 48 MHz clock from 32.768 kHz (ULP32K)
 * The internal FDPLL0 can generate 120 MHz Clock from 32.768 kHz
 * 
 * (diagram generated with asciiflow.com)
 *
 *
 *                                       ┌────────┐ 120M     ┌───────┐  120M    ┌───────┐
 *                                  ┌───►│ FDPLL0 ├─────────►│ GCLK0 ├─────────►│ MCLK  │
 *                                  │    └────────┘          └───────┘          └───────┘
 *                                  │
 *                                  │
 *                                  │    ┌────────┐ 48M      ┌───────┐  48M
 *                                  ├───►│  DFLL  ├─────────►│ GCLK1 ├─────────►
 *                                  │    └────────┘          └───────┘
 * ┌────────┐          ┌───────┐    │
 * │ ULP32K ├─────────►│ GCLK3 ├────┘
 * └────────┘32.768k   └───────┘ 32.768k
 *
 *
 * 
 * Some specific peripheral settings used below are chosen from atmel start
 * Others are chosen based on what Arduino-samd core or micropython do
 * 
 */
void clocks_init(void){
    GCLK->CTRLA.bit.SWRST = 1;                                  // Reset GCLKs
    while(GCLK->SYNCBUSY.bit.SWRST);                            // Wait for reset

    // Configure GCLK3
    GCLK->GENCTRL[3].reg = 
        GCLK_GENCTRL_SRC_OSCULP32K |                            // Select ULP32K as source
        GCLK_GENCTRL_GENEN;                                     // Enable GCLK
    while(GCLK->SYNCBUSY.bit.GENCTRL3);                         // Wait for sync

    // Configure GCLK0 (temporary)
    // Keeps CPU clock alive during other config
    GCLK->GENCTRL[0].reg =  
        GCLK_GENCTRL_SRC_OSCULP32K |                            // Select ULP32K as source
        GCLK_GENCTRL_GENEN;                                     // Enable GCLK
    while(GCLK->SYNCBUSY.bit.GENCTRL0);                         // Wait for sync

    // DFLL configuration (open loop mode)
    GCLK->PCHCTRL[OSCCTRL_GCLK_ID_DFLL48].reg =                 // Configure clock to DFLL
        GCLK_PCHCTRL_CHEN |                                     // Enable GCLK to DFLL
        GCLK_PCHCTRL_GEN_GCLK3;                                 // GCLK3 as DFLL source
    OSCCTRL->DFLLCTRLA.reg = 0x00;                              // Disable before config
    OSCCTRL->DFLLMUL.reg = 
        OSCCTRL_DFLLMUL_CSTEP(0x1) |                            // Coarse maximum step = 1
        OSCCTRL_DFLLMUL_FSTEP(0x1) |                            // Fine maximum step = 1
        OSCCTRL_DFLLMUL_MUL(0);                                 // Multiply factor = 0
    while(OSCCTRL->DFLLSYNC.bit.DFLLMUL);                       // Wait for sync
    OSCCTRL->DFLLCTRLB.reg = 0;                                 // Clear config
    while(OSCCTRL->DFLLSYNC.bit.DFLLCTRLB);                     // Wait for sync
    OSCCTRL->DFLLCTRLA.bit.ENABLE = 1;                          // Enable DFLL
    while(OSCCTRL->DFLLSYNC.bit.ENABLE);                        // Wait for sync
    OSCCTRL->DFLLVAL.reg = OSCCTRL->DFLLVAL.reg;                // Used to wait for sync before continuing
    while(OSCCTRL->DFLLSYNC.bit.DFLLVAL);                       // Wait for sync
    OSCCTRL->DFLLCTRLB.reg = 
        OSCCTRL_DFLLCTRLB_WAITLOCK |                            // Fine lock
        OSCCTRL_DFLLCTRLB_CCDIS;                                // Chill cycle disable (b/c Arduino core does it)
    while(OSCCTRL->DFLLSYNC.bit.DFLLCTRLB);                     // Wait for sync
    while(!OSCCTRL->STATUS.bit.DFLLRDY);                        // Wait for ready

    // Configure GCLK1
    GCLK->GENCTRL[1].reg = 
            GCLK_GENCTRL_SRC_DFLL |                             // Source from DFLL           
            GCLK_GENCTRL_GENEN;                                 // Enable GCLK
    while(GCLK->SYNCBUSY.bit.GENCTRL1);                         // Wait for sync

    // FDPLL0 configuration for 120MHz
    // See page 702 of datasheet for how to calculate
    // loop divider ratio
    // f_out = (3661 + 1 + 4/32) * 32768 = 120000512Hz
    GCLK->PCHCTRL[OSCCTRL_GCLK_ID_FDPLL0].reg =                 // Configure clock to FDPLL0
        GCLK_PCHCTRL_CHEN |                                     // Enable clock to FDPLL0
        GCLK_PCHCTRL_GEN_GCLK3;                                 // Source from GCLK3
    OSCCTRL->Dpll[0].DPLLRATIO.reg = 
        OSCCTRL_DPLLRATIO_LDRFRAC(4) |                          // Loop divider ratio (frac part)
        OSCCTRL_DPLLRATIO_LDR(3661);                            // Loop divider ratio (int part)
    while(OSCCTRL->Dpll[0].DPLLSYNCBUSY.bit.DPLLRATIO);         // Wait for sync
    OSCCTRL->Dpll[0].DPLLCTRLB.reg = 
        OSCCTRL_DPLLCTRLB_REFCLK_GCLK |                         // Reference from GCLK (configured above)
        OSCCTRL_DPLLCTRLB_LBYPASS;                              // Enable if low freq ref clock (see errata)
    OSCCTRL->Dpll[0].DPLLCTRLA.reg = 
        OSCCTRL_DPLLCTRLA_ENABLE;                               // Enable FDPLL0
    while(OSCCTRL->Dpll[0].DPLLSYNCBUSY.bit.ENABLE);            // Wait for sync
    while(!OSCCTRL->Dpll[0].DPLLSTATUS.bit.CLKRDY);             // Wait for ready
    while(!OSCCTRL->Dpll[0].DPLLSTATUS.bit.LOCK);               // Wait for lock

    // GCLK0 Configuration (120MHz now)
    GCLK->GENCTRL[0].reg = 
        GCLK_GENCTRL_SRC_DPLL0 |                                // Source from DPLL0
        GCLK_GENCTRL_IDC |                                      // Better duty cycle for odd division factors
        GCLK_GENCTRL_GENEN;                                     // Enable GCLK0
    while(GCLK->SYNCBUSY.bit.GENCTRL0);                         // Wait for sync

    // MCLK Configuration
    MCLK->CPUDIV.reg = MCLK_CPUDIV_DIV_DIV1;                    // CPU clock = MCLK / 1 = 120MHz

    // Update this variable (in case used elsewhere)
    SystemCoreClock = 120000000;

    delay_init();
}
