/**
 * @file ms5837.c
 * @author Marcus Behel
 */

#include <ms5837.h>
#include <flags.h>
#include <i2c0.h>
#include <timers.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MS5837_ADDR                     0x76

#define MS5837_CMD_RESET                0x1E
#define MS5837_CMD_ADC_READ             0x00
#define MS5837_CMD_PROM_READ            0xA0
#define MS5837_CMD_CONVERT_D1_8192      0x4A
#define MS5837_CMD_CONVERT_D2_8192      0x5A

// Buffer settings
#define WRITE_BUF_SIZE          16          // max number of write bytes per transaction
#define READ_BUF_SIZE           16          // max number of read bytes per transaction

// States
#define STATE_DELAY             0           // State while delay in progress
#define STATE_NONE              1           // Before state machine is started. Used to init state machine
#define STATE_RESET             2           // Write reset command
#define STATE_READ_PROM         3           // Read PROM
#define STATE_CONV_D1           4           // Write D1 convert command (pressure)
#define STATE_READ_ADC          5           // Write read adc command and read data
#define STATE_BAD_SENSOR        6           // Dead end state for invalid sensor version / id

// State transition triggers
#define TRIGGER_NONE            0
#define TRIGGER_I2C_DONE        1
#define TRIGGER_I2C_ERROR       2
#define TRIGGER_DELAY_DONE      3
#define TRIGGER_READ            4


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static uint8_t wbuf[WRITE_BUF_SIZE];
static volatile uint8_t rbuf[READ_BUF_SIZE];
i2c_trans ms5837_trans;

static uint8_t state;
static uint32_t delay;
static uint8_t delay_next_state;

static float depth;
static bool connected;

// Flags that trigger "abnormal" transitions at the next trigger of the state machine
// These will not trigger immediately, but when the current state finishes interrupting
// normal flow of the state machine. However, if the state machine is in an idle state
// (a state where no action is in progress), these should also trigger an immediate
// transition (TRIGGER_NONE).
static bool reset;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// State Machine diagram generated using asciiflow.com
/*
 *               │init || reset
 *               │                                      trigger(delay_ms)
 *        ┌──────▼──────┐                             ────────────────────►
 *        │    RESET    │
 *        └──────┬──────┘                           Note: i2c_error causes
 *               │i2c_done(10)                      a repeat of any state after 10ms
 *               │
 *        ┌──────▼──────┐  i2c_done,!valid
 *        │  READ_PROM  ├───────────────────┐      valid means crc check passes
 *        └──────┬──────┘                   │      and sensor version / id is
 * i2c_done,valid│                          │      correct
 *               │                          │
 *        ┌──────▼──────┐            ┌──────▼──────┐
 *  ┌─────►   CONV_D1   │            │ BAD_SENSOR  │
 *  │     └──────┬──────┘            └─────────────┘
 *  │            │i2c_done(20)
 *  │            │
 *  │     ┌──────▼──────┐
 *  │     │  READ_ADC   │
 *  │     └──────┬──────┘
 *  │            │i2c_done(20)
 *  └────────────┘
 */
static void ms5837_state_machine(uint8_t trigger){
    // Store original state
    uint8_t orig_state = state;

    // -----------------------------------------------------------------------------------------------------------------
    // State changes
    // -----------------------------------------------------------------------------------------------------------------
    
    if(reset){
        // Handle "abnormal" triggers first
        state = STATE_RESET;
        reset = false;
    }else if(trigger == TRIGGER_I2C_ERROR){
        // Repeat same state after 10ms
        delay = 10;
        delay_next_state = state;
        state = STATE_DELAY;
    }else{
        // Other state transitions
        switch(state){
        case STATE_DELAY:
            // Transition out of delay state when delay done
            if(trigger == TRIGGER_DELAY_DONE){
                state = delay_next_state;
            }
            break;
        case STATE_NONE:
            // Always transition to next state
            // This is just used to start the state machine
            state = STATE_RESET;
            break;
        case STATE_RESET:
            if(trigger == TRIGGER_I2C_DONE){
                state = STATE_DELAY;
                delay = 10;
                delay_next_state = STATE_READ_PROM;
            }
            break;
        case STATE_READ_PROM:
            if(trigger == TRIGGER_I2C_DONE){
                bool valid = false; // TODO
                state = valid ? STATE_CONV_D1 : STATE_BAD_SENSOR;
                connected = valid;
            }
            break;
        case STATE_CONV_D1:
            if(trigger == TRIGGER_I2C_DONE){
                state = STATE_DELAY;
                delay = 20;                                         // Max conversion time in datasheet
                delay_next_state = STATE_READ_ADC;
            }
            break;
        case STATE_READ_ADC:
            if(trigger == TRIGGER_I2C_DONE){
                // TODO: Handle read data
                state = STATE_DELAY;
                delay = 20;                                         // Defines rate of reads
                delay_next_state = STATE_CONV_D1;
            }
            break;
        }
    }

    if((state == orig_state)){
        // No state transition occurred
        return;
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Actions at START of state
    // -----------------------------------------------------------------------------------------------------------------

    switch(state){
    case STATE_DELAY:
        timers_ms5837_delay(delay);
        break;
    case STATE_RESET:
        connected = false;
        ms5837_trans.write_buf[0] = MS5837_CMD_RESET;
        ms5837_trans.write_count = 1;
        ms5837_trans.read_count = 0;
        FLAG_SET(flags_main, FLAG_MAIN_MS5837_WANTI2C);
        break;
    case STATE_READ_PROM:
        ms5837_trans.write_buf[0] = MS5837_CMD_PROM_READ;
        ms5837_trans.write_count = 1;
        ms5837_trans.read_count = 0;
        FLAG_SET(flags_main, FLAG_MAIN_MS5837_WANTI2C);
        break;
    case STATE_CONV_D1:
        ms5837_trans.write_buf[0] = MS5837_CMD_CONVERT_D1_8192;
        ms5837_trans.write_count = 1;
        ms5837_trans.read_count = 0;
        FLAG_SET(flags_main, FLAG_MAIN_MS5837_WANTI2C);
        break;
    case STATE_READ_ADC:
        ms5837_trans.write_buf[0] = MS5837_CMD_ADC_READ;
        ms5837_trans.write_count = 1;
        ms5837_trans.read_count = 3;
        FLAG_SET(flags_main, FLAG_MAIN_MS5837_WANTI2C);
        break;
    }
}

bool ms5837_init(void){
    // Initial data
    depth = 999;
    reset = false;
    connected = false;
    
    // Setup transaction
    ms5837_trans.address = MS5837_ADDR;
    ms5837_trans.write_buf = wbuf;
    ms5837_trans.read_buf = rbuf;

    // Set initial state and start state machine
    state = STATE_NONE;
    ms5837_state_machine(TRIGGER_NONE);

    return true;
}

void ms5837_i2c_done(void){
    if(ms5837_trans.status == I2C_STATUS_SUCCESS){
        ms5837_state_machine(TRIGGER_I2C_DONE);
    }else{
        ms5837_state_machine(TRIGGER_I2C_ERROR);
    }
}

void ms5837_delay_done(void){
    ms5837_state_machine(TRIGGER_DELAY_DONE);
}

float ms5837_get(void){
    return depth;
}

void ms5837_reset(void){
    // Use a flag because it may not be possible to transition states now
    // In that case, this should override normal state transitions
    reset = true;

    // If the current state performs no actions
    // this can be triggered now
    // Can't do this if waiting on i2c or delay though
    // In those cases, the reset will trigger when i2c or delay finishes
    if(state == STATE_BAD_SENSOR){
        ms5837_state_machine(TRIGGER_NONE);
    }
}
