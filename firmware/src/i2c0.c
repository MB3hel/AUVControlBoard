/**
 * @file i2c0.c
 * @author Marcus Behel
 */


#include <i2c0.h>
#include <flags.h>
#include <timers.h>
#include <sam.h>
#include <usb.h>
#include <ports.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define I2C0_TIMEOUT    2               // ms

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

i2c_trans *i2c0_curr_trans;

static volatile uint32_t transaction_counter; 
static uint32_t timeouts;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void i2c0_init(void){
    // Initial values
    transaction_counter = 0;
    timeouts = 0;

    // Determined per formulas on page 916 of datasheet
    // Standard mode with BAUDLOW = 0
    // Must make an assumption about Trise
    // The expression for BAUD is solving for BAUD and splitting things up
    // to avoid integer overflow
    #define TRISE               300                                 // In ns
    #define FSCL                100000                              // In Hz
    #define FGCLK               48000000                            // In Hz
    #define BAUD_VAL (FGCLK / 2 / FSCL) - (FGCLK / 1000000 * TRISE / 2000) - 5

    ports_i2c0_fix_sda_low();                                       // In case sensor is holding SDA low after MCU reset

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
    if(I2C0_IDLE){
        usb_writemsg((uint8_t[]){'S', 'T', 'A', 'R', 'T'}, 5);

        // If previous transaction did not timeout, reset the timeout counter
        // This counter is supposed to count number of timeouts in a row
        if(i2c0_curr_trans != NULL && i2c0_curr_trans->status != I2C_STATUS_TIMEOUT){
            timeouts = 0;
        }

        i2c0_curr_trans = trans;
        i2c0_curr_trans->status = I2C_STATUS_BUSY;
        
        if(trans->write_count > 0){
            // Start write phase by setting ADDR register
            // MB interrupt will occur when ready to transmit first byte
            // use transaction_counter to track number of bytes transmitted
            // LSB of address used to indicated write (0) or read (1)
            timers_i2c0_timeout(I2C0_TIMEOUT);
            transaction_counter = 0;
            SERCOM2->I2CM.ADDR.bit.ADDR = i2c0_curr_trans->address << 1 | 0b0;
        }else if (trans->read_count > 0){
            // Start read phase. This will write address
            // SB interrupt will occur after first byte received
            // transaction counter will track number of bytes received
            timers_i2c0_timeout(I2C0_TIMEOUT);
            transaction_counter = 0;
            SERCOM2->I2CM.ADDR.bit.ADDR = i2c0_curr_trans->address << 1 | 0b1;
        }else{
            // Transaction completed successfully
            i2c0_curr_trans->status = I2C_STATUS_SUCCESS;
            FLAG_SET(flags_main, FLAG_MAIN_I2C0_DONE);
        }
    }
    return res;
}

void i2c0_timeout(void){
    timeouts++;

    // Sometimes, noise on the bus during transactions (from something such as touching pins)
    // has caused the bus to "freeze" where both lines are high, but no activity occurs (even when)
    // a transaction is attempted). This is seemingly undetectable by software (no status or error bits
    // get set when this occurs). Thus, if 5 transactions in a row time out, assume this is the case.
    // This is solved by disabling and re-enabling the I2C bus. Presumably, this resets the hardware state
    // machine in the SERCOM peripheral.
    if(timeouts >= 5){
        SERCOM2->I2CM.CTRLA.bit.ENABLE = 0;                     // Disable bus
        while(SERCOM2->I2CM.SYNCBUSY.bit.ENABLE);               // Wait for sync
        ports_i2c0_fix_sda_low();                               // Sometimes gets stuck with SDA low
        SERCOM2->I2CM.CTRLA.bit.ENABLE = 1;                     // Enable bus
        while(SERCOM2->I2CM.SYNCBUSY.bit.ENABLE);               // Wait for sync
        timeouts = 0;                                           // Reset counter
        SERCOM2->I2CM.STATUS.bit.BUSERR = 1;                    // Clear BUSERR error bit
        SERCOM2->I2CM.STATUS.bit.ARBLOST = 1;                   // Clear ARBLOST error bit
        SERCOM2->I2CM.STATUS.bit.RXNACK = 1;                    // Clear RXNACK error bit
        SERCOM2->I2CM.STATUS.bit.BUSSTATE = 0x1;                // Force back to IDLE state
    
        usb_writemsg((uint8_t[]){'R', 'E', 'S', 'E', 'T'}, 5);
    }


    // Transaction has now finished with error status
    i2c0_curr_trans->status = I2C_STATUS_TIMEOUT;
    FLAG_SET(flags_main, FLAG_MAIN_I2C0_DONE);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// IRQ handlers
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void irq_handler(void){  
    // Don't handle interrupts unless a transaction is in progress
    // If transaction not in progress, clear flags and return
    // If interrupts occur when no transaction is in progress, they mean nothing
    // Handling them could cause undesired behavior (such as handling an ERROR interrupt
    // due to noise on an idle line).
    if(I2C0_IDLE){
        SERCOM2->I2CM.INTFLAG.reg |= SERCOM_I2CM_INTFLAG_MASK;
        return;
    }

    if(SERCOM2->I2CM.INTFLAG.bit.MB && SERCOM2->I2CM.STATUS.bit.RXNACK){
        // MB bit is set when NACK received during either TX or RX phase.
        // In either case, this is an error

        timers_i2c0_timeout(0);

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

            timers_i2c0_timeout(I2C0_TIMEOUT);

            SERCOM2->I2CM.DATA.bit.DATA = i2c0_curr_trans->write_buf[transaction_counter];
            transaction_counter++;
            // MB flag is cleared when DATA reg is set so no need to clear manually
        }else{
            // No more data to write.

            if(i2c0_curr_trans->read_count > 0){

                if(i2c0_curr_trans->stop_after_write){
                    SERCOM2->I2CM.CTRLB.bit.ACKACT = 1;
                    SERCOM2->I2CM.CTRLB.bit.CMD = 0x03;
                    while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);
                }

                // Start read phase. This will write address after a repeated start
                // Repeated start b/c no stop done before this (intentional)
                // SB interrupt will occur after first byte received
                // transaction counter will track number of bytes received
                timers_i2c0_timeout(I2C0_TIMEOUT);
                transaction_counter = 0;
                SERCOM2->I2CM.ADDR.bit.ADDR = i2c0_curr_trans->address << 1 | 0b1;
            }else{
                timers_i2c0_timeout(0);

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

            timers_i2c0_timeout(I2C0_TIMEOUT);

            // Send ACK and start read of next byte
            SERCOM2->I2CM.CTRLB.bit.ACKACT = 0;
            SERCOM2->I2CM.CTRLB.bit.CMD = 0x02;
            while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);
            
            // SB flag is cleared when CMD bitfield is set so no need to clear manually
        }else{
            // Done reading

            timers_i2c0_timeout(0);

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

        timers_i2c0_timeout(0);

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
