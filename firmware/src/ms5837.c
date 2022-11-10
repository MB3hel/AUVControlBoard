/**
 * @file ms5837.c
 * @author Marcus Behel
 */

#include <ms5837.h>
#include <flags.h>
#include <i2c0.h>
#include <timers.h>
#include <usb.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MS5837_ADDR                     0x76
#define MS5837_30BA_VER                 0x1A   // Sensor version ID for 30BA

#define MS5837_CMD_RESET                0x1E
#define MS5837_CMD_ADC_READ             0x00
#define MS5837_CMD_PROM_READ            0xA0
#define MS5837_CMD_CONVERT_D1_OSR256    0x40
#define MS5837_CMD_CONVERT_D1_OSR512    0x42
#define MS5837_CMD_CONVERT_D1_OSR1024   0x44
#define MS5837_CMD_CONVERT_D1_OSR2048   0x46
#define MS5837_CMD_CONVERT_D1_OSR4096   0x48
#define MS5837_CMD_CONVERT_D1_OSR8192   0x4A
#define MS5837_CMD_CONVERT_D2_OSR256    0x50
#define MS5837_CMD_CONVERT_D2_OSR512    0x52
#define MS5837_CMD_CONVERT_D2_OSR1024   0x54
#define MS5837_CMD_CONVERT_D2_OSR2048   0x56
#define MS5837_CMD_CONVERT_D2_OSR4096   0x58
#define MS5837_CMD_CONVERT_D2_OSR8192   0x5A

// Buffer settings
#define WRITE_BUF_SIZE          16          // max number of write bytes per transaction
#define READ_BUF_SIZE           16          // max number of read bytes per transaction

// States
#define STATE_DELAY             0           // State while delay in progress
#define STATE_NONE              1           // Before state machine is started. Used to init state machine
#define STATE_RESET             2           // Write reset command
#define STATE_READ_PROM         3           // Read PROM
#define STATE_CONV_D1           4           // Write D1 convert command (pressure)
#define STATE_READ_ADC_D1       5           // Write read adc command and read data
#define STATE_CONV_D2           6           // Write D2 convert command (temperature)
#define STATE_READ_ADC_D2       7           // Write read adc command and read data

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

static ms5837_data data;

static uint8_t state;
static uint32_t delay;
static uint8_t delay_next_state;

// Raw data
static uint32_t d1, d2;
static uint16_t prom_data[8];

static uint32_t last_data;
static uint32_t error_counter;

// Flags that will disrupt normal state machine transitions
// These will not trigger anything now, but will be handled the next time
// something is triggered (eg i2c_done or delay_done)
static bool reset;

// TODO: Make this configurable via pc command
// freshwater = 997.0f
// saltwater = 1029.0f
const static float fluid_density = 997.0f;

// TODO: Make this configurable via pc command
const static float atm_pressure = 101325.0f;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Perform calculations
 * See pages 11-12 of sensor datasheet
 */
static void calculate(void){
    int32_t dT = 0;
	int64_t SENS = 0;
	int64_t OFF = 0;
    int32_t TEMP, TEMP100_DIV_100;
    int32_t P;
	int32_t SENSi = 0;
	int32_t OFFi = 0;
	int32_t Ti = 0;
	int64_t OFF2 = 0;
	int64_t SENS2 = 0;

	// Calculate temperature
	dT = d2 - ((uint32_t)prom_data[5]) * 256l;
    TEMP = 2000l + ((int64_t)dT) * prom_data[6] / 8388608LL;
    
    // Calculate temperature compensated pressure
    SENS = ((int64_t)prom_data[1]) * 32768l + (((int64_t)prom_data[3]) * dT) / 256l;
    OFF = ((int64_t)prom_data[2]) * 65536l + (((int64_t)prom_data[4]) * dT) / 128l;
    
    // Skip this because it would just be recalculated in second order
    // P = (d1 * SENS / (2097152l) - OFF) / 8192l;
	

    // -------------------------------------------------------------------------
	// Second order compensation
	// -------------------------------------------------------------------------
    TEMP100_DIV_100 = TEMP / 100;
    if(TEMP100_DIV_100 < 20){
        // Low temperature
        Ti = (3 * ((int64_t)dT) * ((int64_t)dT)) / 8589934592ll;
        OFFi = (3 * (TEMP - 2000) * (TEMP - 2000)) / 2;
        SENSi = (5 * (TEMP - 2000) * (TEMP - 2000)) / 8;
        if(TEMP100_DIV_100 < -15){
            // Very low temperature
            OFFi = OFFi + 7 * (TEMP + 1500l) * (TEMP + 1500l);
            SENSi = SENSi + 4 * (TEMP + 1500l) * (TEMP + 1500l);
        }
    }else{
        // High temperature
        Ti = 2 * (dT * dT) / 137438953472ll;
        OFFi = ((TEMP - 2000) * (TEMP - 2000)) / 16;
        SENSi = 0;
    }

	OFF2 = OFF-OFFi;
	SENS2 = SENS-SENSi;

	TEMP = (TEMP-Ti);
	P = ((d1 * SENS2) / 2097152l - OFF2) / 8192l;

    // P in mbar * 10
    // TEMP in celsius * 100
    // 1mbar = 100Pa -> P * 10 = Pa
    data.pressure_mbar = P / 10.0f;
    data.temperature_c = TEMP / 100.0f;
    data.depth_m = ((P * 10.0f) - atm_pressure) / (fluid_density * 9.80665);
}

/**
 * crc4 function as defined in sensor datasheet (p10)
 */
static uint8_t crc4(uint16_t *data){
    uint32_t count;
    uint32_t remainder = 0;
    uint8_t bit;
    data[0] = (data[0] & 0x0FFF);
    data[7] = 0;
    for(count = 0; count < 16; ++count){
        if(count & 0x1)  remainder ^= (uint16_t)(data[count >> 1] & 0x00FF);
        else             remainder ^= (uint16_t)(data[count >> 1] >> 8);
        for(bit = 8; bit > 0; --bit){
            if(remainder & 0x8000)  remainder = (remainder << 1) ^ 0x3000;
            else                    remainder = remainder << 1;
        }
    }
    remainder = (remainder >> 12) & 0x000F;
    return remainder ^ 0x00;
}

// State Machine diagram generated using asciiflow.com
/*
 *                            │init || reset
 *   i2c_error(1000)          │                                      trigger(delay_ms)
 *           ┌─────────┬──────▼──────┐                             ────────────────────►
 *           │         │    RESET    │◄───────────────────────────┐
 *           └────────►└──────┬──────┘                            │    Note: i2c_error causes
 *                            │i2c_done(10)                       │    a repeat of any state after 10ms
 *                            │         i2c_error(1000)           │    except where otherwise indicated
 *              ┌─────►┌──────▼──────┐  i2c_done(1000),!valid,i==6│
 * i2c_done,i!=6│      │ READ_PROM_i ├────────────────────────────┘
 *              └──────┤ i=0,1,...,6 │
 *                     └──────┬──────┘              valid means crc check passes
 *              i2c_done,valid│
 *                       i==6 │
 *                     ┌──────▼──────┐
 *                     │    IDLE     │
 *                     └──────┬──────┘
 *                            │read
 *                            │
 *                     ┌──────▼──────┐
 *                ┌───►│   CONV_D1   │
 *                │    └──────┬──────┘
 *                │           │i2c_done(20)
 *                │           │
 *                │    ┌──────▼──────┐
 *                │    │ READ_ADC_D1 │
 *                │    └──────┬──────┘
 *                │           │i2c_done
 *                │           │
 *                │    ┌──────▼──────┐
 *                │    │   CONV_D2   │
 *                │    └──────┬──────┘
 *                │           │i2c_done(20)
 *                │           │
 *                │    ┌──────▼──────┐
 *                │    │ READ_ADC_D2 │
 *                │    └──────┬──────┘
 *                │           │i2c_done(20)
 *                └───────────┘
 */
static void ms5837_state_machine(uint8_t trigger){
    // Store original state
    uint8_t orig_state = state;
    bool repeat_state = false;
    static uint32_t prom_read_i;

    // -----------------------------------------------------------------------------------------------------------------
    // State changes
    // -----------------------------------------------------------------------------------------------------------------
    
    if(reset){
        // This should override normal state transitions if set
        reset = false;
        state = STATE_RESET;
        repeat_state = true;  // just in case it was already in reset state

        // Change this to prevent normal transitions from happening
        trigger = TRIGGER_NONE;
    }

    if(trigger == TRIGGER_I2C_ERROR){
        if(state == STATE_READ_PROM || state == STATE_RESET){
            // Special case for initialization states
            delay = 1000;
            delay_next_state = STATE_RESET;
            state = STATE_DELAY;
        }else{
            // Repeat same state after 10ms
            delay = 10;
            delay_next_state = state;
            state = STATE_DELAY;
        }
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
                prom_read_i = 0;
            }
            break;
        case STATE_READ_PROM:
            if(trigger == TRIGGER_I2C_DONE && prom_read_i == 6){
                prom_data[prom_read_i] = (ms5837_trans.read_buf[0] << 8) | ms5837_trans.read_buf[1];

                // Valid if CRC matches 
                uint8_t crc_read = prom_data[0] >> 12;
                uint8_t crc_calc = crc4(prom_data);
                if(crc_read == crc_calc){
                    state = STATE_CONV_D1;
                }else{
                    state = STATE_DELAY;
                    delay = 1000;
                    delay_next_state = STATE_RESET;
                }
            }else if(trigger == TRIGGER_I2C_DONE){
                // Store word for later
                prom_data[prom_read_i] = (ms5837_trans.read_buf[0] << 8) | ms5837_trans.read_buf[1];

                // Read next byte
                prom_read_i++;
                state = STATE_READ_PROM;
                repeat_state = true;
            }
            break;
        case STATE_CONV_D1:
            if(trigger == TRIGGER_I2C_DONE){
                state = STATE_DELAY;
                delay = 20;                                         // Max conversion time in datasheet
                delay_next_state = STATE_READ_ADC_D1;
            }
            break;
        case STATE_READ_ADC_D1:
            if(trigger == TRIGGER_I2C_DONE){
                d1 = (ms5837_trans.read_buf[0] << 16) | (ms5837_trans.read_buf[1] << 8) | ms5837_trans.read_buf[2];
                state = STATE_CONV_D2;
            }
            break;
        case STATE_CONV_D2:
            if(trigger == TRIGGER_I2C_DONE){
                state = STATE_DELAY;
                delay = 20;                                         // Max conversion time in datasheet
                delay_next_state = STATE_READ_ADC_D2;
            }
            break;
        case STATE_READ_ADC_D2:
            if(trigger == TRIGGER_I2C_DONE){
                d2 = (ms5837_trans.read_buf[0] << 16) | (ms5837_trans.read_buf[1] << 8) | ms5837_trans.read_buf[2];
                calculate();
                last_data = timers_now();

                state = STATE_DELAY;
                delay = 20;                                         // Defines sensor read rate
                delay_next_state = STATE_CONV_D1;
            }
            break;
        }
    }

    if((state == orig_state) && !repeat_state){
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
        ms5837_trans.write_buf[0] = MS5837_CMD_RESET;
        ms5837_trans.write_count = 1;
        ms5837_trans.read_count = 0;
        FLAG_SET(flags_main, FLAG_MAIN_MS5837_WANTI2C);
        break;
    case STATE_READ_PROM:
        ms5837_trans.write_buf[0] = MS5837_CMD_PROM_READ + prom_read_i * 2;
        ms5837_trans.write_count = 1;
        ms5837_trans.read_count = 2;
        FLAG_SET(flags_main, FLAG_MAIN_MS5837_WANTI2C);
        break;
    case STATE_CONV_D1:
        ms5837_trans.write_buf[0] = MS5837_CMD_CONVERT_D1_OSR1024;
        ms5837_trans.write_count = 1;
        ms5837_trans.read_count = 0;
        FLAG_SET(flags_main, FLAG_MAIN_MS5837_WANTI2C);
        break;
    case STATE_READ_ADC_D1:
        ms5837_trans.write_buf[0] = MS5837_CMD_ADC_READ;
        ms5837_trans.write_count = 1;
        ms5837_trans.read_count = 3;
        FLAG_SET(flags_main, FLAG_MAIN_MS5837_WANTI2C);
        break;
    case STATE_CONV_D2:
        ms5837_trans.write_buf[0] = MS5837_CMD_CONVERT_D2_OSR1024;
        ms5837_trans.write_count = 1;
        ms5837_trans.read_count = 0;
        FLAG_SET(flags_main, FLAG_MAIN_MS5837_WANTI2C);
        break;
    case STATE_READ_ADC_D2:
        ms5837_trans.write_buf[0] = MS5837_CMD_ADC_READ;
        ms5837_trans.write_count = 1;
        ms5837_trans.read_count = 3;
        FLAG_SET(flags_main, FLAG_MAIN_MS5837_WANTI2C);
        break;
    }
}

bool ms5837_init(void){
    // Initial data
    data.depth_m = 999;
    data.temperature_c = 999;
    data.pressure_mbar = 999;

    // Last time data was read
    last_data = 65535;

    // Initial count 0 b/c no transactions yet
    error_counter = 0;

    // Setup transaction
    ms5837_trans.address = MS5837_ADDR;
    ms5837_trans.write_buf = wbuf;
    ms5837_trans.read_buf = rbuf;
    ms5837_trans.stop_after_write = true;

    // Set initial state and start state machine
    state = STATE_NONE;
    ms5837_state_machine(TRIGGER_NONE);

    return true;
}

void ms5837_i2c_done(void){
    if(ms5837_trans.status == I2C_STATUS_SUCCESS){
        error_counter = 0;
        ms5837_state_machine(TRIGGER_I2C_DONE);
    }else{
        // If too many transactions fail in a row, the sensor is probably no longer connected
        // Or it may just be in a bad state
        // Either way, reset the state machine so the sensor is either "fixed" or it will be 
        // handled correctly when (re)connected
        error_counter++;
        if(error_counter >= 10){
            error_counter = 0;
            reset = true;
        }
        ms5837_state_machine(TRIGGER_I2C_ERROR);
    }
}

void ms5837_delay_done(void){
    ms5837_state_machine(TRIGGER_DELAY_DONE);
}

ms5837_data ms5837_get(void){
    return data;
}

bool ms5837_connected(void){
    // Connected if got data in the 750ms
    return (timers_now() - last_data) < 750;
}

void ms5837_reset(void){
    reset = true;
}

