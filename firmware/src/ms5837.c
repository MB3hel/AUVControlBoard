/**
 * @file ms5837.c
 * @author Marcus Behel
 */

#include <ms5837.h>
#include <flags.h>
#include <i2c0.h>

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
#define STATE_DELAY             0
#define STATE_NONE              1
// TODO

// State transition triggers
#define TRIGGER_NONE            0
#define TRIGGER_I2C_DONE        1
#define TRIGGER_I2C_ERROR       2
#define TRIGGER_DELAY_DONE      3


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


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void ms5837_state_machine(uint8_t trigger){
    // TODO
}

bool ms5837_init(void){
    // Initial data
    depth = 9e99f;
    
    // Setup transaction
    ms5837_trans.address = MS5837_ADDR;
    ms5837_trans.write_buf = wbuf;
    ms5837_trans.read_buf = rbuf;

    // Reset device.
    ms5837_trans.write_buf[0] = MS5837_CMD_RESET;
    ms5837_trans.write_count = 1;
    ms5837_trans.read_count = 0;
    i2c0_start(&ms5837_trans);
    while(!FLAG_CHECK(flags_main, FLAG_MAIN_I2C0_DONE));
    FLAG_CLEAR(flags_main, FLAG_MAIN_I2C0_DONE);
    if(ms5837_trans.status != I2C_STATUS_SUCCESS)
        return false;

    // TODO: Read PROM data and verify it is correct

    

    // Set initial state and start state machine
    state = STATE_NONE;
    ms5837_state_machine(TRIGGER_NONE);

    return true;
}

void ms5837_i2c_done(void){
    // TODO
}

void ms5837_delay_done(void){
    // TODO
}

float ms5837_get(void){
    // TODO
}
