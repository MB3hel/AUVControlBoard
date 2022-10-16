/**
 * @file i2c0.c
 * @author Marcus Behel
 * 
 * Notes: There seem to be numerous bugs stemming from the ASF4 hpl_sercom layer. 
 * It is likely other protocols have bugs too. Seemingly, ASF4 drivers were just written and
 * tested with one test case (the example code) and really don't always work well in practice.
 * 
 * Most of the time, these "bugs" relate to flags not being cleared properly while handling
 * interrupts. This can lead to an interrupt repeating forever (thus deadlocking main)
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

    // Enable the sercom device and force BUSSTATE to idle
    hri_sercomi2cm_set_CTRLA_ENABLE_bit(SERCOM2);
    hri_sercomi2cm_set_STATUS_BUSSTATE_bf(SERCOM2, 0x01);
}

void i2c0_process(void){
    switch(state){
    case STATE_WRITE:
        // Set address
        // TODO: Set slave address

        if(queue[current]->write_count > 0){
            // TODO: Start write operation
        }else{
            // Nothing to write, skip to read state
            state = STATE_READ;
            FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);
        }
        break;
    case STATE_READ:
        if(queue[current]->read_count > 0){
            // TODO: Start read
        }else{
            // Nothing to read, skip to idle state
            state = STATE_IDLE;
            FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);
        }
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

        // TODO: Handle data received

        // TODO: When all requested bytes received
        // state = STATE_IDLE;
        // result = I2C_STATUS_SUCCESS;
        // FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);

        // Clear interrupt flag
        hri_sercomi2cm_clear_INTFLAG_SB_bit(SERCOM2);
    }else if(hri_sercomi2cm_get_INTFLAG_MB_bit(SERCOM2)){
        // MB bit is set when data transmit is done

        // TODO: Handle transmit complete

        // TODO: When all bytes transmitted
        // state = STATE_READ;
        // FLAG_SET(flags_main, FLAG_MAIN_I2C0_PROC);

        // Clear interrupt flag
        hri_sercomi2cm_clear_INTFLAG_MB_bit(SERCOM2);
    }else{
        // This is an ERROR interrupt
        
        // Send STOP
        hri_sercomi2cm_set_CTRLB_CMD_bf(SERCOM2, 0x03);

        // Move back to idle state with error status
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