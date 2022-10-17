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
#include <atmel_start.h>

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
volatile uint32_t txr_counter;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void i2c0_init(void){
    // Initialize queue indices
    current = 0;
    count = 0;
    next = 0;

    // Initial state
    state = STATE_IDLE;

    // Enable SERCOM2 interrupts at controller level
    // In Family datasheet see
    // Page 67 for NVIC mapping
    // Pages 925-926 for I2C mode interrupt description
    // Page 954 for INTFLAG bits
    NVIC_EnableIRQ(SERCOM2_0_IRQn);             // MB = bit 0 in INTFLAG = SERCOM2_0
    NVIC_EnableIRQ(SERCOM2_1_IRQn);             // SB = bit 1 in INTFLAG = SERCOM2_1
    NVIC_EnableIRQ(SERCOM2_2_IRQn);             // Not used in I2CM mode
    NVIC_EnableIRQ(SERCOM2_3_IRQn);             // ERROR = bit 7 in INTFLAG = SERCOM2_7 is handled by SERCOM2_3

    // Enable the sercom device and force BUSSTATE to idle
    hri_sercomi2cm_set_CTRLA_ENABLE_bit(SERCOM2);
    hri_sercomi2cm_write_STATUS_BUSSTATE_bf(SERCOM2, 0x01);
}

void i2c0_process(void){
    switch(state){
    case STATE_WRITE:
        txr_counter = 0;
        // Set slave address (which starts transaction)
        // Will set MB flag when finished. First byte is transmitted there
        // Addresses are 7-bit. LSB is 0 for write, 1 for read
        hri_sercomi2cm_write_ADDR_ADDR_bf(SERCOM2, queue[current]->address << 1 | 0b0);
        break;
    case STATE_READ:
        txr_counter = 0;
        // Set slave address (which starts transaction)
        // Will set SB flag when finished. First byte is read there
        // Addresses are 7-bit. LSB is 0 for write, 1 for read
        hri_sercomi2cm_write_ADDR_ADDR_bf(SERCOM2, queue[current]->address << 1 | 0b1);
        break;
    case STATE_IDLE:
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
            timers_wdt_feed();
        }
    }
}

void i2c0_enqueue(i2c_trans *trans){
    // This should never happen
    // If it does, either queue is too small or i2c is too slow
    if(count == QUEUE_SIZE)
        return;
    
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


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// IRQ handler functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void irq_handler(void){    
    // Common IRQ handler for any SERCOM interrupts for this SERCOM
    // Flags are used to determine what the interrupt is, not which IRQ runs
    if(hri_sercomi2cm_get_INTFLAG_SB_bit(SERCOM2)){
        // SB bit is set when data is received

        // Handle received byte
        if(txr_counter < queue[current]->read_count){
            queue[current]->read_buf[txr_counter] = hri_sercomi2cm_read_DATA_reg(SERCOM2);
            txr_counter++;
        }

        // ACK if another byte is needed
        // NACK if no more bytes should be read
        if(txr_counter == queue[current]->read_count){
            // Send ACK and start read of next byte
            hri_sercomi2cm_clear_CTRLB_ACKACT_bit(SERCOM2);
            hri_sercomi2cm_write_CTRLB_CMD_bf(SERCOM2, 0x02);

            // DO NOT clear SB flag here or it will end transmit
        }else{
            // Send NACK (done reading)
            hri_sercomi2cm_set_CTRLB_ACKACT_bit(SERCOM2);

            // Send STOP (done with transaction)
            hri_sercomi2cm_write_CTRLB_CMD_bf(SERCOM2, 0x03);
        
            // Move to IDLE state (current transaction done)
            state = STATE_IDLE;
            result = I2C_STATUS_SUCCESS;
            FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);

            // Clear interrupt flag only if no more data to read
            hri_sercomi2cm_clear_INTFLAG_SB_bit(SERCOM2);
        }
    }else if(hri_sercomi2cm_get_INTFLAG_MB_bit(SERCOM2)){
        if(hri_sercomi2cm_get_STATUS_RXNACK_bit(SERCOM2)){
            // MB bit may be set on NACK received during either TX or RX
            // This is an ERROR
            // Send STOP
            hri_sercomi2cm_write_CTRLB_CMD_bf(SERCOM2, 0x03);

            // Move to idle state with error status
            state = STATE_IDLE;
            result = I2C_STATUS_ERROR;
            FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);

            // Clear interrupt flag only if no more data to transmit
            hri_sercomi2cm_clear_INTFLAG_MB_bit(SERCOM2);
        }else{
            // Otherwise, MB bit is set when data transmit is done
            
            if(txr_counter < queue[current]->write_count){
                // There is another byte to write
                hri_sercomi2cm_write_DATA_reg(SERCOM2, queue[current]->write_buf[txr_counter]);
                txr_counter++;

                // DO NOT clear MB flag here or it will end transmit
            }else{
                // There are no more bytes to write
                // Move to read state
                state = STATE_READ;
                FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);

                // Clear interrupt flag only if no more data to transmit
                hri_sercomi2cm_clear_INTFLAG_MB_bit(SERCOM2);
            }
        }
    }else{
        // This is an ERROR interrupt
        
        // Send STOP
        hri_sercomi2cm_write_CTRLB_CMD_bf(SERCOM2, 0x03);

        // Move to idle state with error status
        state = STATE_IDLE;
        result = I2C_STATUS_ERROR;
        FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);

        // Clear interrupt flag
        hri_sercomi2cm_clear_INTFLAG_ERROR_bit(SERCOM2);
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