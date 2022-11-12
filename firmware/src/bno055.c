/**
 * @file bno055.c
 * @author Marcus Behel
 */

#include <bno055.h>
#include <i2c0.h>
#include <flags.h>
#include <timers.h>
#include <usb.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Sensor info
#define BNO055_ADDR                         (0x28)
#define BNO055_ID                           (0xA0)

// BNO055 Registers
/* Page id register definition*/
#define BNO055_PAGE_ID_ADDR                 (0X07)

/* PAGE0 REGISTER DEFINITION START*/
#define BNO055_CHIP_ID_ADDR                 (0x00)
#define BNO055_ACCEL_REV_ID_ADDR            (0x01)
#define BNO055_MAG_REV_ID_ADDR              (0x02)
#define BNO055_GYRO_REV_ID_ADDR             (0x03)
#define BNO055_SW_REV_ID_LSB_ADDR           (0x04)
#define BNO055_SW_REV_ID_MSB_ADDR           (0x05)
#define BNO055_BL_REV_ID_ADDR               (0X06)

/* Accel data register*/
#define BNO055_ACCEL_DATA_X_LSB_ADDR        (0X08)
#define BNO055_ACCEL_DATA_X_MSB_ADDR        (0X09)
#define BNO055_ACCEL_DATA_Y_LSB_ADDR        (0X0A)
#define BNO055_ACCEL_DATA_Y_MSB_ADDR        (0X0B)
#define BNO055_ACCEL_DATA_Z_LSB_ADDR        (0X0C)
#define BNO055_ACCEL_DATA_Z_MSB_ADDR        (0X0D)

/*Mag data register*/
#define BNO055_MAG_DATA_X_LSB_ADDR          (0X0E)
#define BNO055_MAG_DATA_X_MSB_ADDR          (0X0F)
#define BNO055_MAG_DATA_Y_LSB_ADDR          (0X10)
#define BNO055_MAG_DATA_Y_MSB_ADDR          (0X11)
#define BNO055_MAG_DATA_Z_LSB_ADDR          (0X12)
#define BNO055_MAG_DATA_Z_MSB_ADDR          (0X13)

/*Gyro data registers*/
#define BNO055_GYRO_DATA_X_LSB_ADDR         (0X14)
#define BNO055_GYRO_DATA_X_MSB_ADDR         (0X15)
#define BNO055_GYRO_DATA_Y_LSB_ADDR         (0X16)
#define BNO055_GYRO_DATA_Y_MSB_ADDR         (0X17)
#define BNO055_GYRO_DATA_Z_LSB_ADDR         (0X18)
#define BNO055_GYRO_DATA_Z_MSB_ADDR         (0X19)

/*Euler data registers*/
#define BNO055_EULER_H_LSB_ADDR             (0X1A)
#define BNO055_EULER_H_MSB_ADDR             (0X1B)

#define BNO055_EULER_R_LSB_ADDR             (0X1C)
#define BNO055_EULER_R_MSB_ADDR             (0X1D)

#define BNO055_EULER_P_LSB_ADDR             (0X1E)
#define BNO055_EULER_P_MSB_ADDR             (0X1F)

/*Quaternion data registers*/
#define BNO055_QUATERNION_DATA_W_LSB_ADDR   (0X20)
#define BNO055_QUATERNION_DATA_W_MSB_ADDR   (0X21)
#define BNO055_QUATERNION_DATA_X_LSB_ADDR   (0X22)
#define BNO055_QUATERNION_DATA_X_MSB_ADDR   (0X23)
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR   (0X24)
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR   (0X25)
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR   (0X26)
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR   (0X27)

/* Linear acceleration data registers*/
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR (0X28)
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR (0X29)
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR (0X2A)
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR (0X2B)
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR (0X2C)
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR (0X2D)

/*Gravity data registers*/
#define BNO055_GRAVITY_DATA_X_LSB_ADDR      (0X2E)
#define BNO055_GRAVITY_DATA_X_MSB_ADDR      (0X2F)
#define BNO055_GRAVITY_DATA_Y_LSB_ADDR      (0X30)
#define BNO055_GRAVITY_DATA_Y_MSB_ADDR      (0X31)
#define BNO055_GRAVITY_DATA_Z_LSB_ADDR      (0X32)
#define BNO055_GRAVITY_DATA_Z_MSB_ADDR      (0X33)

/* Temperature data register*/
#define BNO055_TEMP_ADDR                    (0X34)

/* Status registers*/
#define BNO055_CALIB_STAT_ADDR              (0X35)
#define BNO055_SELFTEST_RESULT_ADDR         (0X36)
#define BNO055_INTR_STAT_ADDR               (0X37)
#define BNO055_SYS_CLK_STAT_ADDR            (0X38)
#define BNO055_SYS_STAT_ADDR                (0X39)
#define BNO055_SYS_ERR_ADDR                 (0X3A)

/* Unit selection register*/
#define BNO055_UNIT_SEL_ADDR                (0X3B)
#define BNO055_DATA_SELECT_ADDR             (0X3C)

/* Mode registers*/
#define BNO055_OPR_MODE_ADDR                (0X3D)
#define BNO055_PWR_MODE_ADDR                (0X3E)

#define BNO055_SYS_TRIGGER_ADDR             (0X3F)
#define BNO055_TEMP_SOURCE_ADDR             (0X40)

/* Axis remap registers*/
#define BNO055_AXIS_MAP_CONFIG_ADDR         (0X41)
#define BNO055_AXIS_MAP_SIGN_ADDR           (0X42)

/* SIC registers*/
#define BNO055_SIC_MATRIX_0_LSB_ADDR        (0X43)
#define BNO055_SIC_MATRIX_0_MSB_ADDR        (0X44)
#define BNO055_SIC_MATRIX_1_LSB_ADDR        (0X45)
#define BNO055_SIC_MATRIX_1_MSB_ADDR        (0X46)
#define BNO055_SIC_MATRIX_2_LSB_ADDR        (0X47)
#define BNO055_SIC_MATRIX_2_MSB_ADDR        (0X48)
#define BNO055_SIC_MATRIX_3_LSB_ADDR        (0X49)
#define BNO055_SIC_MATRIX_3_MSB_ADDR        (0X4A)
#define BNO055_SIC_MATRIX_4_LSB_ADDR        (0X4B)
#define BNO055_SIC_MATRIX_4_MSB_ADDR        (0X4C)
#define BNO055_SIC_MATRIX_5_LSB_ADDR        (0X4D)
#define BNO055_SIC_MATRIX_5_MSB_ADDR        (0X4E)
#define BNO055_SIC_MATRIX_6_LSB_ADDR        (0X4F)
#define BNO055_SIC_MATRIX_6_MSB_ADDR        (0X50)
#define BNO055_SIC_MATRIX_7_LSB_ADDR        (0X51)
#define BNO055_SIC_MATRIX_7_MSB_ADDR        (0X52)
#define BNO055_SIC_MATRIX_8_LSB_ADDR        (0X53)
#define BNO055_SIC_MATRIX_8_MSB_ADDR        (0X54)

/* Accelerometer Offset registers*/
#define BNO055_ACCEL_OFFSET_X_LSB_ADDR      (0X55)
#define BNO055_ACCEL_OFFSET_X_MSB_ADDR      (0X56)
#define BNO055_ACCEL_OFFSET_Y_LSB_ADDR      (0X57)
#define BNO055_ACCEL_OFFSET_Y_MSB_ADDR      (0X58)
#define BNO055_ACCEL_OFFSET_Z_LSB_ADDR      (0X59)
#define BNO055_ACCEL_OFFSET_Z_MSB_ADDR      (0X5A)

/* Magnetometer Offset registers*/
#define BNO055_MAG_OFFSET_X_LSB_ADDR        (0X5B)
#define BNO055_MAG_OFFSET_X_MSB_ADDR        (0X5C)
#define BNO055_MAG_OFFSET_Y_LSB_ADDR        (0X5D)
#define BNO055_MAG_OFFSET_Y_MSB_ADDR        (0X5E)
#define BNO055_MAG_OFFSET_Z_LSB_ADDR        (0X5F)
#define BNO055_MAG_OFFSET_Z_MSB_ADDR        (0X60)

/* Gyroscope Offset registers*/
#define BNO055_GYRO_OFFSET_X_LSB_ADDR       (0X61)
#define BNO055_GYRO_OFFSET_X_MSB_ADDR       (0X62)
#define BNO055_GYRO_OFFSET_Y_LSB_ADDR       (0X63)
#define BNO055_GYRO_OFFSET_Y_MSB_ADDR       (0X64)
#define BNO055_GYRO_OFFSET_Z_LSB_ADDR       (0X65)
#define BNO055_GYRO_OFFSET_Z_MSB_ADDR       (0X66)

/* Radius registers*/
#define BNO055_ACCEL_RADIUS_LSB_ADDR        (0X67)
#define BNO055_ACCEL_RADIUS_MSB_ADDR        (0X68)
#define BNO055_MAG_RADIUS_LSB_ADDR          (0X69)
#define BNO055_MAG_RADIUS_MSB_ADDR          (0X6A)

/* PAGE0 REGISTERS DEFINITION END*/
/* PAGE1 REGISTERS DEFINITION START*/
/* Configuration registers*/
#define BNO055_ACCEL_CONFIG_ADDR            (0X08)
#define BNO055_MAG_CONFIG_ADDR              (0X09)
#define BNO055_GYRO_CONFIG_ADDR             (0X0A)
#define BNO055_GYRO_MODE_CONFIG_ADDR        (0X0B)
#define BNO055_ACCEL_SLEEP_CONFIG_ADDR      (0X0C)
#define BNO055_GYRO_SLEEP_CONFIG_ADDR       (0X0D)
#define BNO055_MAG_SLEEP_CONFIG_ADDR        (0x0E)

/* Interrupt registers*/
#define BNO055_INT_MASK_ADDR                (0X0F)
#define BNO055_INT_ADDR                     (0X10)
#define BNO055_ACCEL_ANY_MOTION_THRES_ADDR  (0X11)
#define BNO055_ACCEL_INTR_SETTINGS_ADDR     (0X12)
#define BNO055_ACCEL_HIGH_G_DURN_ADDR       (0X13)
#define BNO055_ACCEL_HIGH_G_THRES_ADDR      (0X14)
#define BNO055_ACCEL_NO_MOTION_THRES_ADDR   (0X15)
#define BNO055_ACCEL_NO_MOTION_SET_ADDR     (0X16)
#define BNO055_GYRO_INTR_SETING_ADDR        (0X17)
#define BNO055_GYRO_HIGHRATE_X_SET_ADDR     (0X18)
#define BNO055_GYRO_DURN_X_ADDR             (0X19)
#define BNO055_GYRO_HIGHRATE_Y_SET_ADDR     (0X1A)
#define BNO055_GYRO_DURN_Y_ADDR             (0X1B)
#define BNO055_GYRO_HIGHRATE_Z_SET_ADDR     (0X1C)
#define BNO055_GYRO_DURN_Z_ADDR             (0X1D)
#define BNO055_GYRO_ANY_MOTION_THRES_ADDR   (0X1E)
#define BNO055_GYRO_ANY_MOTION_SET_ADDR     (0X1F)

// Operating modes
#define OPMODE_CFG              0x00
#define OPMODE_IMU              0X08

// Buffer settings
#define WRITE_BUF_SIZE          16          // max number of write bytes per transaction
#define READ_BUF_SIZE           16          // max number of read bytes per transaction

// States
#define STATE_DELAY             0           // Delay state
#define STATE_NONE              1           // No state (used for init)
#define STATE_EXIST_CHECK       2           // Read and validate chip id register
#define STATE_SETMODE_CFG       3           // Set to config mode
#define STATE_RESET             4           // Reset chip
#define STATE_RD_ID             5           // Read chip id
#define STATE_WR_PWR_MODE       6           // Set power mode
#define STATE_WR_PAGE_ID        7           // Write page id register
#define STATE_WR_SYSTRIGGER     8           // Write sys trigger register
#define STATE_WR_AXIS_MAP       9           // Set axis map
#define STATE_WR_AXIS_SIGN      10           // Set axis signs
#define STATE_SETMODE_IMU       11          // Set to imu mode
#define STATE_RD_GRAV           12          // Read gravity vector
#define STATE_RD_EULER          13          // Read euler orientation
#define STATE_RECONFIG          14          // Enter config mode before reconfigure

// State transition triggers
#define TRIGGER_NONE            0
#define TRIGGER_I2C_DONE        1
#define TRIGGER_I2C_ERROR       2
#define TRIGGER_DELAY_DONE      3


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// TODO: Remove
static uint32_t stuck_counter;

static uint8_t axis_config;
static bool reconfig;

static uint8_t wbuf[WRITE_BUF_SIZE];
static volatile uint8_t rbuf[READ_BUF_SIZE];

i2c_trans bno055_trans;

static uint8_t state;

static uint32_t delay;
static uint8_t delay_next_state;

static bno055_data data;

static bool reset;

static uint32_t last_data;
static uint32_t error_counter;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// State machine diagram generated with asciiflow.com
/*
 * i2c_error(1000)                  │init || reset
 * i2c_done(1000),invalid_id        │
 *           ┌──────────────┬───────▼───────┐
 *           │              │  EXIST_CHECK  │                   trigger(delay_ms)
 *           └─────────────►└───────┬───────┘                 ─────────────────────►
 *                                  │i2c_done(10),valid_id
 *                                  │                         Note: i2c_error causes
 *                          ┌───────▼───────┐                 a repeat of any state after 10ms
 *                          │  SETMODE_CFG  │                 except where otherwise indicated
 *                          └───────┬───────┘
 *                                  │i2c_done(30)
 *                                  │
 *                          ┌───────▼───────┐
 *                          │     RESET     │
 *                          └───────┬───────┘
 *                                  │i2c_done(30)
 *                                  │
 *                          ┌───────▼───────┐◄───────┐
 *                          │     RD_ID     │        │i2c_done,invalid_id(10)
 *                          └───────┬───────┴────────┘
 *                                  │i2c_done,valid_id(50)
 *                                  │
 *                          ┌───────▼───────┐
 *                          │  WR_PWR_MODE  │
 *                          └───────┬───────┘
 *                                  │i2c_done(10)
 *                                  │
 *                          ┌───────▼───────┐
 *                          │  WR_PAGE_ID   │
 *                          └───────┬───────┘
 *                                  │i2c_done(10)
 *                                  │
 *                          ┌───────▼───────┐
 *                          │ WR_SYSTRIGGER │
 *                          └───────┬───────┘
 *                                  │i2c_done(10)
 *                                  │
 *                          ┌───────▼───────┐
 *                          │  WR_AXIS_MAP  ◄─────────────────────────────────┐
 *                          └───────┬───────┘                                 │
 *                                  │i2c_done(10)                             │
 *                                  │                                         │
 *                          ┌───────▼───────┐                                 │
 *                          │ WR_AXIS_SIGN  │                                 │
 *                          └───────┬───────┘                                 │
 *                                  │i2c_done(10)                             │
 *                                  │                                         │
 *                          ┌───────▼───────┐                                 │
 *                          │  SETMODE_IMU  │                                 │
 *                          └───────┬───────┘                                 │
 *                                  │i2c_done(20)                             │
 *                                  │                             i2c_done(30)│
 *                          ┌───────▼───────┐                         ┌───────┴───────┐
 *       ┌──────────────────►    RD_GRAV    │                         │   RECONFIG    │
 *       │                  └───────┬───────┘                         └───────▲───────┘
 *       │                          │i2c_done(15)                             │
 *       │                          │                                         │
 *       │                  ┌───────▼───────┐                                 │
 *       │                  │    RD_EULER   │                                 │
 *       │                  └───┬───────┬───┘                                 │
 *       │                      │       │                                     │
 *       │                      │       │                                     │
 *       │i2c_done,!reconfig(15)│       │i2c_done,reconfig(15)                │
 *       │                      │       │                                     │
 *       │                      │       │                                     │
 *       │                      │       │                                     │
 *       └──────────────────────┘       └─────────────────────────────────────┘
 */
// Called when some event that may change state occurs
static void bno055_state_machine(uint8_t trigger){
    int16_t tmp16;
    bool repeat_state = false;
    
    // -----------------------------------------------------------------------------------------------------------------
    // State changes
    // -----------------------------------------------------------------------------------------------------------------
    
    // Store original state
    uint8_t orig_state = state;

    if(reset){
        // This should override normal state transitions if set
        reset = false;
        state = STATE_EXIST_CHECK;
        repeat_state = true;  // just in case it was already in reset state

        // Change this to prevent normal transitions from happening
        trigger = TRIGGER_NONE;
    }

    if(trigger == TRIGGER_I2C_ERROR){
        // Repeat same state after 10ms (1000ms if exist check)
        delay = (state == STATE_EXIST_CHECK) ? 1000 : 10;
        delay_next_state = state;
        state = STATE_DELAY;
    }else{
        // State transitions
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
            state = STATE_EXIST_CHECK;
            break;
        case STATE_EXIST_CHECK:
            if(trigger == TRIGGER_I2C_DONE){
                if(bno055_trans.read_buf[0] == BNO055_ID){
                    state = STATE_DELAY;
                    delay = 10;
                    delay_next_state = STATE_SETMODE_CFG;
                }else{
                    state = STATE_DELAY;
                    delay = 1000;
                    delay_next_state = STATE_EXIST_CHECK;
                }
            }
            break;
        case STATE_SETMODE_CFG:
            if(trigger == TRIGGER_I2C_DONE){
                state = STATE_DELAY;
                delay = 30;
                delay_next_state = STATE_RESET;
            }
            break;
        case STATE_RESET:
            if(trigger == TRIGGER_I2C_DONE){
                state = STATE_DELAY;
                delay = 30;
                delay_next_state = STATE_RD_ID;
            }
            break;
        case STATE_RD_ID:
            if(trigger == TRIGGER_I2C_DONE){
                if(bno055_trans.read_buf[0] != BNO055_ID){
                    // Invalid id. Read again until correct id.
                    state = STATE_DELAY;
                    delay = 10;
                    delay_next_state = STATE_RD_ID;
                }else{
                    // Valid id. Move to next state
                    state = STATE_DELAY;
                    delay = 10;
                    delay_next_state = STATE_WR_PWR_MODE;
                }
            }
            break;
        case STATE_WR_PWR_MODE:
            if(trigger == TRIGGER_I2C_DONE){
                state = STATE_DELAY;
                delay = 10;
                delay_next_state = STATE_WR_PAGE_ID;
            }
            break;
        case STATE_WR_PAGE_ID:
            if(trigger == TRIGGER_I2C_DONE){
                state = STATE_DELAY;
                delay = 10;
                delay_next_state = STATE_WR_SYSTRIGGER;
            }
            break;
        case STATE_WR_SYSTRIGGER:
            if(trigger == TRIGGER_I2C_DONE){
                state = STATE_DELAY;
                delay = 10;
                delay_next_state = STATE_WR_AXIS_MAP;
            }
            break;
        case STATE_WR_AXIS_MAP:
            if(trigger == TRIGGER_I2C_DONE){
                state = STATE_DELAY;
                delay = 10;
                delay_next_state = STATE_WR_AXIS_SIGN;
            }
            break;
        case STATE_WR_AXIS_SIGN:
            if(trigger == TRIGGER_I2C_DONE){
                state = STATE_DELAY;
                delay = 10;
                delay_next_state = STATE_SETMODE_IMU;
            }
            break;
        case STATE_SETMODE_IMU:
            if(trigger == TRIGGER_I2C_DONE){
                state = STATE_DELAY;
                delay = 20;
                delay_next_state = STATE_RD_GRAV;
            }
            break;
        case STATE_RD_GRAV:
            if(trigger == TRIGGER_I2C_DONE){
                tmp16 = ((int16_t)bno055_trans.read_buf[0]) | (((int16_t)bno055_trans.read_buf[1]) << 8);
                data.grav_x = tmp16 / 100.0f;
                tmp16 = ((int16_t)bno055_trans.read_buf[2]) | (((int16_t)bno055_trans.read_buf[3]) << 8);
                data.grav_y = tmp16 / 100.0f;
                tmp16 = ((int16_t)bno055_trans.read_buf[4]) | (((int16_t)bno055_trans.read_buf[5]) << 8);
                data.grav_z = -tmp16 / 100.0f;

                state = STATE_DELAY;
                delay = 15;
                delay_next_state = STATE_RD_EULER;
            }
            break;
        case STATE_RD_EULER:
            if(trigger == TRIGGER_I2C_DONE){
                tmp16 = ((int16_t)bno055_trans.read_buf[0]) | (((int16_t)bno055_trans.read_buf[1]) << 8);
                data.euler_yaw = tmp16 / 16.0f;
                tmp16 = ((int16_t)bno055_trans.read_buf[2]) | (((int16_t)bno055_trans.read_buf[3]) << 8);
                data.euler_roll = tmp16 / 16.0f;
                tmp16 = ((int16_t)bno055_trans.read_buf[4]) | (((int16_t)bno055_trans.read_buf[5]) << 8);
                data.euler_pitch = tmp16 / 16.0f;

                last_data = timers_now();

                state = STATE_DELAY;
                delay = 15;
                delay_next_state = reconfig ? STATE_RECONFIG : STATE_RD_GRAV;
                reconfig = false;
            }
            break;
        case STATE_RECONFIG:
            if(trigger == TRIGGER_I2C_DONE){
                state = STATE_DELAY;
                delay = 30;
                delay_next_state = STATE_WR_AXIS_MAP;
            }
            break;
        }
    }

    if((state == orig_state) && !repeat_state){
        // No state transition occurred
        return;
    }

    stuck_counter = 0;

    // -----------------------------------------------------------------------------------------------------------------
    // Actions at START of state
    // -----------------------------------------------------------------------------------------------------------------

    switch(state){
    case STATE_DELAY:
        timers_bno055_delay(delay);
        break;
    case STATE_EXIST_CHECK:
        bno055_trans.write_buf[0] = BNO055_CHIP_ID_ADDR;
        bno055_trans.write_count = 1;
        bno055_trans.read_count = 1;
        FLAG_SET(flags_main, FLAG_MAIN_BNO055_WANTI2C);
        break;
    case STATE_SETMODE_CFG:
        bno055_trans.write_buf[0] = BNO055_OPR_MODE_ADDR;
        bno055_trans.write_buf[1] = OPMODE_CFG;
        bno055_trans.write_count = 2;
        bno055_trans.read_count = 0;
        FLAG_SET(flags_main, FLAG_MAIN_BNO055_WANTI2C);
        break;
    case STATE_RESET:
        bno055_trans.write_buf[0] = BNO055_SYS_TRIGGER_ADDR;
        bno055_trans.write_buf[1] = 0x20;
        bno055_trans.write_count = 2;
        bno055_trans.read_count = 0;
        FLAG_SET(flags_main, FLAG_MAIN_BNO055_WANTI2C);
        break;
    case STATE_RD_ID:
        bno055_trans.write_buf[0] = BNO055_CHIP_ID_ADDR;
        bno055_trans.write_count = 1;
        bno055_trans.read_count = 1;
        FLAG_SET(flags_main, FLAG_MAIN_BNO055_WANTI2C);
        break;
    case STATE_WR_PWR_MODE:
        bno055_trans.write_buf[0] = BNO055_PWR_MODE_ADDR;
        bno055_trans.write_buf[1] = 0x00; // Normal power mode
        bno055_trans.write_count = 2;
        bno055_trans.read_count = 0;
        FLAG_SET(flags_main, FLAG_MAIN_BNO055_WANTI2C);
        break;
    case STATE_WR_PAGE_ID:
        bno055_trans.write_buf[0] = BNO055_PAGE_ID_ADDR;
        bno055_trans.write_buf[1] = 0x00;
        bno055_trans.write_count = 2;
        bno055_trans.read_count = 0;
        FLAG_SET(flags_main, FLAG_MAIN_BNO055_WANTI2C);
        break;
    case STATE_WR_SYSTRIGGER:
        bno055_trans.write_buf[0] = BNO055_SYS_TRIGGER_ADDR;
        bno055_trans.write_buf[1] = 0x00;
        bno055_trans.write_count = 2;
        bno055_trans.read_count = 0;
        FLAG_SET(flags_main, FLAG_MAIN_BNO055_WANTI2C);
        break;
    case STATE_WR_AXIS_MAP:
        bno055_trans.write_buf[0] = BNO055_AXIS_MAP_CONFIG_ADDR;
        switch(axis_config){
        case BNO055_AXIS_REMAP_P0:
            bno055_trans.write_buf[1] = 0x21;
            break;
        case BNO055_AXIS_REMAP_P1:
            bno055_trans.write_buf[1] = 0x24;
            break;
        case BNO055_AXIS_REMAP_P2:
            bno055_trans.write_buf[1] = 0x24;
            break;
        case BNO055_AXIS_REMAP_P3:
            bno055_trans.write_buf[1] = 0x21;
            break;
        case BNO055_AXIS_REMAP_P4:
            bno055_trans.write_buf[1] = 0x24;
            break;
        case BNO055_AXIS_REMAP_P5:
            bno055_trans.write_buf[1] = 0x21;
            break;
        case BNO055_AXIS_REMAP_P6:
            bno055_trans.write_buf[1] = 0x21;
            break;
        case BNO055_AXIS_REMAP_P7:
            bno055_trans.write_buf[1] = 0x24;
            break;
        }
        bno055_trans.write_count = 2;
        bno055_trans.read_count = 0;
        FLAG_SET(flags_main, FLAG_MAIN_BNO055_WANTI2C);
        break;
    case STATE_WR_AXIS_SIGN:
        bno055_trans.write_buf[0] = BNO055_AXIS_MAP_SIGN_ADDR;
        switch(axis_config){
        case BNO055_AXIS_REMAP_P0:
            bno055_trans.write_buf[1] = 0x04;
            break;
        case BNO055_AXIS_REMAP_P1:
            bno055_trans.write_buf[1] = 0x00;
            break;
        case BNO055_AXIS_REMAP_P2:
            bno055_trans.write_buf[1] = 0x06;
            break;
        case BNO055_AXIS_REMAP_P3:
            bno055_trans.write_buf[1] = 0x02;
            break;
        case BNO055_AXIS_REMAP_P4:
            bno055_trans.write_buf[1] = 0x03;
            break;
        case BNO055_AXIS_REMAP_P5:
            bno055_trans.write_buf[1] = 0x01;
            break;
        case BNO055_AXIS_REMAP_P6:
            bno055_trans.write_buf[1] = 0x07;
            break;
        case BNO055_AXIS_REMAP_P7:
            bno055_trans.write_buf[1] = 0x05;
            break;
        }
        bno055_trans.write_count = 2;
        bno055_trans.read_count = 0;
        FLAG_SET(flags_main, FLAG_MAIN_BNO055_WANTI2C);
        break;
    case STATE_SETMODE_IMU:
        bno055_trans.write_buf[0] = BNO055_OPR_MODE_ADDR;
        bno055_trans.write_buf[1] = OPMODE_IMU;
        bno055_trans.write_count = 2;
        bno055_trans.read_count = 0;
        FLAG_SET(flags_main, FLAG_MAIN_BNO055_WANTI2C);
        break;
    case STATE_RD_GRAV:
        bno055_trans.write_buf[0] = BNO055_GRAVITY_DATA_X_LSB_ADDR;
        bno055_trans.write_count = 1;
        bno055_trans.read_count = 6;
        FLAG_SET(flags_main, FLAG_MAIN_BNO055_WANTI2C);
        break;
    case STATE_RD_EULER:
        bno055_trans.write_buf[0] = BNO055_EULER_H_LSB_ADDR;
        bno055_trans.write_count = 1;
        bno055_trans.read_count = 6;
        FLAG_SET(flags_main, FLAG_MAIN_BNO055_WANTI2C);
        break;
    case STATE_RECONFIG:
        bno055_trans.write_buf[0] = BNO055_OPR_MODE_ADDR;
        bno055_trans.write_buf[1] = OPMODE_CFG;
        bno055_trans.write_count = 2;
        bno055_trans.read_count = 0;
        FLAG_SET(flags_main, FLAG_MAIN_BNO055_WANTI2C);
        break;
    }
}

void bno055_init(void){
    // Initial data
    data.grav_x = 0.0f;
    data.grav_y = 0.0f;
    data.grav_z = 0.0f;
    data.euler_pitch = 0.0f;
    data.euler_roll = 0.0f;
    data.euler_yaw = 0.0f;

    // Initial value
    error_counter = 0;
    stuck_counter = 0;

    // Initial flags
    reconfig = false;

    // Setup initial config
    axis_config = BNO055_AXIS_REMAP_P5;
    last_data = 65535;
    
    // Setup transaction
    bno055_trans.address = BNO055_ADDR;
    bno055_trans.write_buf = wbuf;
    bno055_trans.read_buf = rbuf;
    bno055_trans.stop_after_write = false;

    // Set initial state and start state machine
    state = STATE_NONE;
    bno055_state_machine(TRIGGER_NONE);
}

void bno055_i2c_done(void){
    if(bno055_trans.status == I2C_STATUS_SUCCESS){
        error_counter = 0;
        bno055_state_machine(TRIGGER_I2C_DONE);
    }else{
        // If too many transactions fail in a row, the sensor is probably no longer connected
        // Or it may just be in a bad state
        // Either way, reset the state machine so the sensor is either "fixed" or it will be 
        // handled correctly when (re)connected
        error_counter++;
        switch(state){
        case STATE_RD_ID:
            // Waiting for sensor to be ready by reading ID register
            // This is expected to fail MANY times, so raise the error counter threshold massively
            if(error_counter >= 1000){
                error_counter = 0;
                reset = true;
            }
            break;
        default:
            if(error_counter >= 10){
                error_counter = 0;
                reset = true;
            }
            break;
        }
        bno055_state_machine(TRIGGER_I2C_ERROR);
    }
}

void bno055_delay_done(void){
    bno055_state_machine(TRIGGER_DELAY_DONE);
}

bno055_data bno055_get(void){
    return data;
}

bool bno055_connected(void){
    // Connected if got data in the 750ms
    return (timers_now() - last_data) < 750;
}

// TODO: Remove this and debug the actual problem
void bno055_fix_stuck(void){
    if(state == STATE_DELAY){
        stuck_counter+= 50;
        // If delay is double what it should have been, force exit delay now
        if((delay < 100 && stuck_counter >= 200) || (delay >= 100 && stuck_counter >= (2 * delay))){
            usb_debugmsg("IMU_FIX");
            timers_bno055_delay(0);
            FLAG_CLEAR(flags_main, FLAG_MAIN_BNO055_DELAY);
            bno055_delay_done();
        }
    }
}
