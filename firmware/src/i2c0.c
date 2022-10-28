/**
 * @file i2c0.c
 * @author Marcus Behel
 * 
 * Notes on ASF4's hri nomenclature
 * 
 * There are various functions to perform actions on bits, bitfields, or registers
 * These actions are
 * set = Set some bits to a 1 (using a mask; bit indices in the mask that are 1 will be set)
 * get = Get the value of some bits (using a mask; bit indices in the mask that are 1 will be read)
 * clear = Set some bits to a 0 (using a mask; bit indices in the mask that are 1 will be cleared)
 * write = Copy a given value to the bit, bitfield, or register
 * read = Read the entire bit, bitfield, or register
 * 
 * Note that set / clear / get for single bits take no mask argument (because there is only one bit)
 * Thus
 * set_bit is the same as write_bit(1)
 * get_bit is the same as read_bit
 * clear_bit is the same as write_bit(0)
 * 
 * Also not that masks should not be shifted. This means that the lowest index of the bitfield is 
 * bit 0 in the mask (regardless of what bit it is in the register).
 * 
 */


#include <i2c0.h>
#include <flags.h>
#include <timers.h>
#include <clocks.h>
#include <sam.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// States
#define STATE_IDLE          0                                   // Idle, doing nothing
#define STATE_WRITE         1                                   // Write a byte
#define STATE_READ          2                                   // Read a byte

#define QUEUE_SIZE          6

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static volatile uint8_t state;
static volatile uint8_t result;

// Transaction queue (implemented as ring buffer)
// next = index to insert at next
// current = index of current transaction
// count = number of items in queue
static i2c_trans *queue[QUEUE_SIZE];
static uint16_t next, current, count;

// Transceiver counter
// Tracks number of bytes either transmitted or received
// During TX and RX phases
static volatile uint32_t txr_counter;

static volatile uint32_t repeated_timeouts;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void i2c0_timeout(void);

void i2c0_init(void){
    // Clock configuration
    GCLK->PCHCTRL[SERCOM2_GCLK_ID_CORE].bit.GEN = CLOCKS_GCLK_120M;
    GCLK->PCHCTRL[SERCOM2_GCLK_ID_CORE].bit.CHEN = 1;
    GCLK->PCHCTRL[SERCOM2_GCLK_ID_SLOW].bit.GEN = CLOCKS_GCLK_32K;
    GCLK->PCHCTRL[SERCOM2_GCLK_ID_SLOW].bit.CHEN = 1;

    MCLK->APBBMASK.bit.SERCOM2_ = 1;

    // Setup of sercom for I2C0
    SERCOM2->I2CM.CTRLA.bit.ENABLE = 0;                 // Disable SERCOm
    while(SERCOM2->I2CM.SYNCBUSY.bit.ENABLE);           // Wait for sync
    SERCOM2->I2CM.CTRLA.bit.SWRST = 1;                  // Software reset
    while(SERCOM2->I2CM.SYNCBUSY.bit.SWRST);            // Wait for reset
    SERCOM2->I2CM.CTRLA.bit.SDAHOLD = 0x03;             // SDA Hold time
    SERCOM2->I2CM.CTRLA.bit.MODE = 0x05;                // I2CM Mode
    SERCOM2->I2CM.CTRLA.bit.SPEED = 0;                  // Set speed field (standard mode)
    SERCOM2->I2CM.BAUD.reg = 0x25E5B;                   // Configure baud rate (100kHz data rate)
    SERCOM2->I2CM.INTFLAG.reg = 
            SERCOM_I2CM_INTFLAG_MB |                    // Enable MB interrupt
            SERCOM_I2CM_INTFLAG_SB |                    // Enable SB interrupt
            SERCOM_I2CM_INTFLAG_ERROR;                  // Enable ERROR interrupt

    // Enable IRQ Handlers
    NVIC_EnableIRQ(SERCOM2_0_IRQn);                     // MB = bit 0 in INTFLAG = SERCOM2_0
    NVIC_EnableIRQ(SERCOM2_1_IRQn);                     // SB = bit 1 in INTFLAG = SERCOM2_1
    NVIC_EnableIRQ(SERCOM2_2_IRQn);                     // Not used in I2CM mode
    NVIC_EnableIRQ(SERCOM2_3_IRQn);                     // ERROR = bit 7 in INTFLAG = SERCOM2_7 is handled by SERCOM2_3

    // Enable the sercom device and force BUSSTATE to idle
    SERCOM2->I2CM.CTRLA.bit.ENABLE = 1;
    while(SERCOM2->I2CM.SYNCBUSY.bit.ENABLE);
    SERCOM2->I2CM.STATUS.bit.BUSSTATE = 0x01;

    // Setup timeout task
    // This timeout is for receiving or transmitting one byte
    // The counter is enabled in i2c0_process either WRITE or READ state
    // It is reset each time a transmit / receive finishes (irq handler)
    // except when no more to tx / rx in which case it is disabled
    timers_i2c0_timeout_init(&i2c0_timeout, 500);
    repeated_timeouts = 0;

    // Initialize queue indices
    current = 0;
    count = 0;
    next = 0;

    // Initial state
    state = STATE_IDLE;
}

void i2c0_process(void){
    switch(state){
    case STATE_WRITE:
        // Enable timeout counter
        timers_i2c0_timeout_reset();

        txr_counter = 0;
        // Set slave address (which starts transaction)
        // Will set MB flag when finished. First byte is transmitted there
        // Addresses are 7-bit. LSB is 0 for write, 1 for read
        SERCOM2->I2CM.ADDR.bit.ADDR = queue[current]->address << 1 | 0b0;
        break;
    case STATE_READ:
        // Enable timeout counter
        timers_i2c0_timeout_reset();

        txr_counter = 0;
        // Set slave address (which starts transaction)
        // Will set SB flag when finished. First byte is read there
        // Addresses are 7-bit. LSB is 0 for write, 1 for read
        SERCOM2->I2CM.ADDR.bit.ADDR = queue[current]->address << 1 | 0b1;
        break;
    case STATE_IDLE:
        if(result == I2C_STATUS_ERROR){

            // Clear error bits
            SERCOM2->I2CM.STATUS.bit.BUSERR = 1;
            SERCOM2->I2CM.STATUS.bit.ARBLOST = 1;
            SERCOM2->I2CM.STATUS.bit.RXNACK = 1;

            // Hardware I2C state machine seems to not handle some errors properly
            // mostly related to induced noise. Thus, sometimes it gets "stuck"
            // where no error is known from software, but there is no activity on i2c pins.
            // If 5 transactions fail in a row, assume this is the case and reset i2c sercom.
            if(repeated_timeouts >= 5){
                SERCOM2->I2CM.CTRLA.bit.ENABLE = 0;
                while(SERCOM2->I2CM.SYNCBUSY.bit.ENABLE);
                asm("nop");
                SERCOM2->I2CM.CTRLA.bit.ENABLE = 1;
                while(SERCOM2->I2CM.SYNCBUSY.bit.ENABLE);
                repeated_timeouts = 0;
            }
        }else{
            repeated_timeouts = 0;
        }

        // Force bus to idle state
        SERCOM2->I2CM.STATUS.bit.BUSSTATE = 0x01;

        // Set result and flag
        queue[current]->status = result;
        FLAG_SET(flags_main, FLAG_MAIN_I2C0_DONE);

        // Adjust queue vars
        current++;
        if(current == QUEUE_SIZE)
            current = 0;
        count--;

        // Start next transaction if queue not empty
        if(count > 0){
            state = STATE_WRITE;
            FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);
        }
        break;
    }
}

void i2c0_perform(i2c_trans *trans){
    // Add transaction to queue (cannot just start or it may interrupt a current transaction)
    i2c0_enqueue(trans);

    // Wait for this transaction to finish (anything else in the queue will run first)
    while(trans->status == I2C_STATUS_BUSY){
        if(FLAG_CHECK(flags_main, FLAG_MAIN_I2C0_PROC)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_I2C0_PROC);
            i2c0_process();
        }else if(FLAG_CHECK(flags_main, FLAG_MAIN_10MS)){
            FLAG_CLEAR(flags_main, FLAG_MAIN_10MS);
            TIMERS_WDT_FEED();
        }
    }
}

void i2c0_enqueue(i2c_trans *trans){
    // This should never happen
    // If it does, either queue is too small or i2c is too slow
    if(count == QUEUE_SIZE){
        return;
    }
    
    // Add trans to queue
    trans->status = I2C_STATUS_BUSY;
    queue[next] = trans;
    next++;
    if(next == QUEUE_SIZE)
        next = 0;
    count++;

    // If idle, start this transaction now
    if(state == STATE_IDLE){
        state = STATE_WRITE;
        FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);
    }
}

void i2c0_timeout(void){
    // Timeout counter is automatically disabled before this is called (see timers.c)

    repeated_timeouts++;

    // Send STOP
    SERCOM2->I2CM.CTRLB.bit.CMD = 0x03;
    while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);

    // Move to idle state with error status
    state = STATE_IDLE;
    result = I2C_STATUS_ERROR;
    FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// IRQ handler functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void irq_handler(void){  
    // Common IRQ handler for any SERCOM interrupts for this SERCOM
    // Flags are used to determine what the interrupt is, not which IRQ runs
    if(SERCOM2->I2CM.INTFLAG.bit.SB){
        // SB bit is set when data is received

        // Handle received byte
        if(txr_counter < queue[current]->read_count){
            queue[current]->read_buf[txr_counter] = SERCOM2->I2CM.DATA.reg;
            txr_counter++;
        }

        // ACK if another byte is needed
        // NACK if no more bytes should be read
        if(txr_counter < queue[current]->read_count){
            // Reset timeout counter
            // TOOD: timers_i2c0_timeout_reset();

            // Send ACK and start read of next byte
            SERCOM2->I2CM.CTRLB.bit.ACKACT = 0;
            SERCOM2->I2CM.CTRLB.bit.CMD = 0x02;
            // SB flag is cleared when CMD bitfield is set (see pg 954 of datasheet)
        }else{
            // Receive done. Disable timeout counter.
            timers_i2c0_timeout_disable();

            // Set ACK action to NACK (done reading)
            SERCOM2->I2CM.CTRLB.bit.ACKACT = 1;

            // Send ACK action followed by STOP (done with transaction)
            // This also clears the SB flag because CMD bitfield is set
            SERCOM2->I2CM.CTRLB.bit.CMD = 0x03;
            while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);
        
            // Move to IDLE state (current transaction done)
            state = STATE_IDLE;
            result = I2C_STATUS_SUCCESS;
            FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);
        }
    }else if(SERCOM2->I2CM.INTFLAG.bit.MB){
        if(SERCOM2->I2CM.STATUS.bit.RXNACK){
            // Disable timeout counter
            timers_i2c0_timeout_disable();

            // MB bit may be set on NACK received during either TX or RX
            // This is an ERROR
            // Send STOP
            SERCOM2->I2CM.CTRLB.bit.CMD = 0x03;
            while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);

            // Move to idle state with error status
            state = STATE_IDLE;
            result = I2C_STATUS_ERROR;
            FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);

            // Clear interrupt flag manually
            SERCOM2->I2CM.INTFLAG.reg |= SERCOM_I2CM_INTFLAG_MB;
        }else{
            // Otherwise, MB bit is set when data transmit is done
            
            if(txr_counter < queue[current]->write_count){
                // Reset timeout counter
                timers_i2c0_timeout_reset();

                // There is another byte to write
                SERCOM2->I2CM.DATA.bit.DATA = queue[current]->write_buf[txr_counter];
                txr_counter++;
                // MB flag is cleared when DATA register is set (see pg 954 of datasheet)
            }else{
                // Disable timeout counter
                timers_i2c0_timeout_disable();

                // There are no more bytes to write
                // Move to read state
                state = STATE_READ;
                FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);

                // Clear interrupt flag manually
                SERCOM2->I2CM.INTFLAG.reg |= SERCOM_I2CM_INTFLAG_MB;
            }
        }
    }else{
        // This is an ERROR interrupt
        
        // Disable timeout counter
        timers_i2c0_timeout_disable();

        // Send STOP
        SERCOM2->I2CM.CTRLB.bit.CMD = 0x03;
        while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);

        // Move to idle state with error status
        state = STATE_IDLE;
        result = I2C_STATUS_ERROR;
        FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);

        // Clear interrupt flag manually
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