/**
 * @file i2c0.c
 * @author Marcus Behel
 */


#include <i2c0.h>
#include <flags.h>
#include <timers.h>
#include <sam.h>


// TODO: I2C timeout using timers

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

i2c_trans *i2c0_curr_trans;

static volatile uint32_t transaction_counter; 


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void i2c0_init(void){
    // Initial values
    transaction_counter = 0;

    // Determined per formulas on page 916 of datasheet
    // Standard mode with BAUDLOW = 0
    // Must make an assumption about Trise
    // The expression for BAUD is solving for BAUD and splitting things up
    // to avoid integer overflow
    #define TRISE               300                                 // In ns
    #define FSCL                100000                              // In Hz
    #define FGCLK               48000000                            // In Hz
    #define BAUD_VAL (FGCLK / 2 / FSCL) - (FGCLK / 1000000 * TRISE / 2000) - 5

    MCLK->APBBMASK.bit.SERCOM2_ = 1;                                // Enable APB clock to SERCOM2

    SERCOM2->I2CM.CTRLA.bit.ENABLE = 0;                             // Disable I2C0
    while(SERCOM2->I2CM.SYNCBUSY.bit.ENABLE);                       // Wait for sync
    SERCOM2->I2CM.CTRLA.bit.SWRST = 1;                              // Reset SERCOM2
    while(SERCOM2->I2CM.SYNCBUSY.bit.SWRST);                        // Wait for reset
    SERCOM2->I2CM.CTRLA.bit.SDAHOLD = 0x03;                         // Max hold time
    SERCOM2->I2CM.CTRLA.bit.MODE = 0x05;                            // I2C Master mode
    SERCOM2->I2CM.CTRLA.bit.SPEED = 0x00;                           // Standard I2C mode
    SERCOM2->I2CM.CTRLB.bit.QCEN = 0;                               // Disable quick command
    SERCOM2->I2CM.CTRLB.bit.SMEN = 0;                               // Disable smart mode
    SERCOM2->I2CM.BAUD.reg = BAUD_VAL;                              // Set baud
    SERCOM2->I2CM.INTFLAG.reg |= SERCOM_I2CM_INTFLAG_MASK;          // Clear all interrupt flags
    SERCOM2->I2CM.INTENSET.bit.MB = 1;                              // Enable MB interrupt
    SERCOM2->I2CM.INTENSET.bit.SB = 1;                              // Enable SB interrupt
    SERCOM2->I2CM.INTENSET.bit.ERROR = 1;                           // Enable ERROR interrupt
    NVIC_EnableIRQ(SERCOM2_0_IRQn);                                 // Enable SERCOM2 IRQ Handlers
    NVIC_EnableIRQ(SERCOM2_1_IRQn);                                 // Enable SERCOM2 IRQ Handlers
    NVIC_EnableIRQ(SERCOM2_2_IRQn);                                 // Enable SERCOM2 IRQ Handlers
    NVIC_EnableIRQ(SERCOM2_3_IRQn);                                 // Enable SERCOM2 IRQ Handlers
    SERCOM2->I2CM.CTRLA.bit.ENABLE = 1;                             // Enable SERCOM
    while(SERCOM2->I2CM.SYNCBUSY.bit.ENABLE);                       // Wait for sync
    SERCOM2->I2CM.STATUS.bit.BUSSTATE = 0x01;                       // Force bus to IDLE state
}

bool i2c0_start(i2c_trans *trans){
    bool res = false;
    // Have to disable interrupts to prevent scenarios where
    // transaction finishes (by IRQ handler) while running below code
    __disable_irq();
    if(I2C0_IDLE){
        i2c0_curr_trans = trans;
        i2c0_curr_trans->status = I2C_STATUS_BUSY;
        
        if(trans->write_count > 0){
            // Start write phase by setting ADDR register
            // MB interrupt will occur when ready to transmit first byte
            // use transaction_counter to track number of bytes transmitted
            // LSB of address used to indicated write (0) or read (1)
            transaction_counter = 0;
            SERCOM2->I2CM.ADDR.bit.ADDR = i2c0_curr_trans->address << 1 | 0b0;
        }else if (trans->read_count > 0){
            // Start read phase. This will write address
            // SB interrupt will occur after first byte received
            // transaction counter will track number of bytes received
            transaction_counter = 0;
            SERCOM2->I2CM.ADDR.bit.ADDR = i2c0_curr_trans->address << 1 | 0b1;
        }else{
            // Transaction completed successfully
            i2c0_curr_trans->status = I2C_STATUS_SUCCESS;
            FLAG_SET(flags_main, FLAG_MAIN_I2C0_DONE);
        }
    }
    __enable_irq();
    return res;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// IRQ handlers
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void irq_handler(void){  
    if(SERCOM2->I2CM.INTFLAG.bit.MB && SERCOM2->I2CM.STATUS.bit.RXNACK){
        // MB bit is set when NACK received during either TX or RX phase.
        // In either case, this is an error

        // Send STOP
        SERCOM2->I2CM.CTRLB.bit.CMD = 0x03;
        while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);

        // Transaction has now finished with error status
        i2c0_curr_trans->status = I2C_STATUS_ERROR;
        FLAG_SET(flags_main, FLAG_MAIN_I2C0_DONE);

        // Manually clear interrupt flag in this case
        SERCOM2->I2CM.INTFLAG.reg |= SERCOM_I2CM_INTFLAG_MB;
    }else if(SERCOM2->I2CM.INTFLAG.bit.MB){
        // MB bit is set once a byte has been transmitted
        
        if(transaction_counter < i2c0_curr_trans->write_count){
            // There is another byte to write
            SERCOM2->I2CM.DATA.bit.DATA = i2c0_curr_trans->write_buf[transaction_counter];
            transaction_counter++;
            // MB flag is cleared when DATA reg is set so no need to clear manually
        }else{
            // No more data to write.

            if(i2c0_curr_trans->read_count > 0){
                // Start read phase. This will write address after a repeated start
                // Repeated start b/c no stop done before this (intentional)
                // SB interrupt will occur after first byte received
                // transaction counter will track number of bytes received
                transaction_counter = 0;
                SERCOM2->I2CM.ADDR.bit.ADDR = i2c0_curr_trans->address << 1 | 0b1;
            }else{
                // No data to read. Stop transaction now.
                // Send STOP
                SERCOM2->I2CM.CTRLB.bit.CMD = 0x03;
                while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);

                // Transaction completed successfully
                i2c0_curr_trans->status = I2C_STATUS_SUCCESS;
                FLAG_SET(flags_main, FLAG_MAIN_I2C0_DONE);
            }
        }

    }else if(SERCOM2->I2CM.INTFLAG.bit.SB){
        // SB bit is set once a byte has been received
        
        // Copy read byte
        i2c0_curr_trans->read_buf[transaction_counter] = SERCOM2->I2CM.DATA.reg;
        transaction_counter++;

        if(transaction_counter < i2c0_curr_trans->read_count){
            // Another byte should be read

            // Send ACK and start read of next byte
            SERCOM2->I2CM.CTRLB.bit.ACKACT = 0;
            SERCOM2->I2CM.CTRLB.bit.CMD = 0x02;
            while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);
            
            // SB flag is cleared when CMD bitfield is set so no need to clear manually
        }else{
            // Done reading

            // Send NACK and STOP
            SERCOM2->I2CM.CTRLB.bit.ACKACT = 1;
            SERCOM2->I2CM.CTRLB.bit.CMD = 0x03;
            while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);

            // Transaction completed successfully
            i2c0_curr_trans->status = I2C_STATUS_SUCCESS;
            FLAG_SET(flags_main, FLAG_MAIN_I2C0_DONE);

            // SB flag is cleared when CMD bitfield is set so no need to clear manually
        }

    }else if(SERCOM2->I2CM.INTFLAG.bit.ERROR){
        // ERROR bit is set when an error occurs

        // Send STOP
        SERCOM2->I2CM.CTRLB.bit.CMD = 0x03;
        while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);

        // Transaction has now finished with error status
        i2c0_curr_trans->status = I2C_STATUS_ERROR;
        FLAG_SET(flags_main, FLAG_MAIN_I2C0_DONE);

        // Manually clear interrupt flag in this case
        SERCOM2->I2CM.INTFLAG.reg |= SERCOM_I2CM_INTFLAG_ERROR;
    }
}

void SERCOM2_0_Handler(void){
    irq_handler();
}

void SERCOM2_1_Handler(void){
    irq_handler();
}

void SERCOM2_2_Handler(void){
    irq_handler();
}

void SERCOM2_3_Handler(void){
    irq_handler();
}
