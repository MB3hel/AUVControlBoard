
#include <bno055.h>
#include <i2c0.h>
#include <stdint.h>
#include <timers.h>


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

// States
#define STATE_NONE                          255
#define STATE_CFG_START                     0           // Set into config mode to start config
#define STATE_CFG_RST                       1           // Reset IMU
#define STATE_DELAY                         2           // Delay for some duration
#define STATE_CFG_ID                        3           // Check sensor ID
#define STATE_CFG_PWR                       4           // Configure power mode
#define STATE_CFG_PAGE                      5           // Set PAGE_ID_ADDR
#define STATE_CFG_AXRMP                     6           // Set axis remap
#define STATE_CFG_AXSGN                     7           // Set axis signs
#define STATE_CFG_MODE                      8           // Set operating mode
#define STATE_IDLE                          9           // Idle (delay between readings)
#define STATE_RECONFIG                      10          // Back into config mode (reconfigure axes)
#define STATE_READ_GRAV                     11          // Read gravity vector
#define STATE_READ_QUAT                     12          // Read orientation quaternion

// Operating modes
#define OPMODE_CFG                          0b0000
#define OPMODE_IMU                          0b1000

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Current I2C transaction for this sensor
static i2c_trans curr_trans;

// Read and write buffers for this transaction
static uint8_t write_buf[16];
static uint8_t read_buf[16];

// Delay counter and state to transition to when delay done
static uint32_t delay = 0;
static uint8_t delay_next_state;

// Current state
static uint8_t curr_state;

// Reconfig flag
static bool reconfig = false;

// Current axis config
static bno055_axis_config axis_config;

// Current reading data
static bno055_data data;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void bno055_state_machine(bool i2c_done, bool delay_done){
    /*
    *                       │init
    *                       │
    *                ┌──────▼──────┐
    *                │  CFG_START  │
    *                └──────┬──────┘
    *                       │i2c_done
    *                       │
    *                ┌──────▼──────┐
    *                │    DELAY    │ (30ms)
    *                └──────┬──────┘
    *                       │delay_done
    *                       │
    *                ┌──────▼──────┐
    *                │   CFG_RST   │
    *                └──────┬──────┘
    *                       │i2c_done
    *                       │
    *                ┌──────▼──────┐
    *                │    DELAY    │ (30ms)
    *                └──────┬──────┘
    *                       │delay_done
    *                       │
    *                ┌──────▼──────┐◄───────┐
    *                │   CFG_ID    │        │i2c_done,!valid
    *                └──────┬──────┴────────┘
    *         i2c_done,valid│
    *                       │
    *                ┌──────▼──────┐
    *                │    DELAY    │ (50ms)
    *                └──────┬──────┘
    *                       │delay_done
    *                       │
    *                ┌──────▼──────┐
    *                │   CFG_PWR   │
    *                └──────┬──────┘
    *                       │i2c_done
    *                       │
    *                ┌──────▼──────┐
    *                │    DELAY    │ (10ms)
    *                └──────┬──────┘
    *                       │delay_done
    *                       │
    *                ┌──────▼──────┐
    *                │  CFG_PAGE   │
    *                └──────┬──────┘
    *                       │i2c_done
    *                       │
    *                ┌──────▼──────┐
    *                │    DELAY    │ (10ms)
    *                └──────┬──────┘
    *                       │delay_done
    *                       │
    *                ┌──────▼──────┐
    *                │  CFG_AXRMP  ◄──────────────────────────────┐
    *                └──────┬──────┘                              │
    *                       │i2c_done                             │
    *                       │                                     │
    *                ┌──────▼──────┐                              │
    *                │    DELAY    │ (10ms)                       │
    *                └──────┬──────┘                              │
    *                       │delay_done                           │
    *                       │                                     │
    *                ┌──────▼──────┐                              │
    *                │  CFG_AXSGN  │                              │
    *                └──────┬──────┘                              │
    *                       │i2c_done                             │
    *                       │                                     │
    *                ┌──────▼──────┐                              │
    *                │    DELAY    │ (10ms)                       │
    *                └──────┬──────┘                              │
    *                       │delay_done                           │
    *                       │                                     │
    *                ┌──────▼──────┐                              │
    *                │  CFG_MODE   │                              │
    *                └──────┬──────┘                              │
    *                       │i2c_done                             │
    *                       │                                     │
    *                ┌──────▼──────┐                              │
    *                │    DELAY    │ (20ms)                       │
    *                └──────┬──────┘                              │
    *                       │delay_done                           │
    *                       │                                     │
    *                ┌──────▼──────┐                              │
    *           ┌───►│  READ_GRAV  │                              │
    *           │    └──────┬──────┘                              │
    *           │           │i2c_done                             │
    *           │           │                                     │
    *           │    ┌──────▼──────┐                              │
    *           │    │  READ_QUAT  │                              │
    *           │    └──────┬──────┘                              │
    *           │           │i2c_done                             │
    *           │           │                                     │
    *           │           │                                     │
    *           │           │                                     │
    * delay_done│    ┌──────▼──────┐                              │
    *           │    │    IDLE     │ (10ms)                       │
    *           │    └──┬────────┬─┘                              │
    *           │       │        │reconfig                        │
    *           └───────┘        │                                │
    *                            │                                │
    *                ┌───────────▼─┐                              │
    *                │  RECONFIG   │                              │
    *                └──────┬──────┘                              │
    *                       │i2c_done                             │
    *                       │                                     │
    *                ┌──────▼──────┐                              │
    *                │    DELAY    │ (30ms)                       │
    *                └──────┬──────┘                              │
    *                       │                                     │
    *                       └─────────────────────────────────────┘
    *                            delay_done
    */
    
    // State machine is implemented in 2 parts
    // State transitions are handled first
    // Then, if the new state is not the same, state actions are performed for the new state
    // Finally, the current state is transitioned to the new state

    // When this function is called, the arguments are "flags" that trigger transitions
    // Additionally, the reconfig flag is global and holds its value until handled
    // This allows a reconfig to be requested during any state, even though it will not be handled
    // until the IDLE state

    // Make sure this is STATE_NONE if no transition occurs
    uint8_t next_state = STATE_NONE;

    // State transitions
    switch(curr_state){
    case STATE_NONE:
        // Always transition into CFG_START (first call from init)
        next_state = STATE_CFG_START;
        break;
    case STATE_CFG_START:
        if(i2c_done){
            next_state = STATE_DELAY;
            delay = 30;
            delay_next_state = STATE_CFG_RST;
        }
        break;
    case STATE_CFG_RST:
        if(i2c_done){
            next_state = STATE_DELAY;
            delay = 30;
            delay_next_state = STATE_CFG_ID;
        }
        break;
    case STATE_DELAY:
        if(delay_done){
            next_state = delay_next_state;
        }
        break;
    case STATE_CFG_ID:
        if(i2c_done){
            bool valid = curr_trans.read_buf[0] == BNO055_ID;
            valid = valid && (curr_trans.status == I2C_STATUS_SUCCESS);
            if(valid){
                next_state = STATE_DELAY;
                delay = 50;
                delay_next_state = STATE_CFG_PWR;
            }else{
                next_state = STATE_CFG_ID;
            }
        }
        break;
    case STATE_CFG_PWR:
        if(i2c_done){
            next_state = STATE_DELAY;
            delay = 10;
            delay_next_state = STATE_CFG_PAGE;
        }
        break;
    case STATE_CFG_PAGE:
        if(i2c_done){
            next_state = STATE_DELAY;
            delay = 10;
            delay_next_state = STATE_CFG_AXRMP;
        }
        break;
    case STATE_CFG_AXRMP:
        if(i2c_done){
            next_state = STATE_DELAY;
            delay = 10;
            delay_next_state = STATE_CFG_AXSGN;
        }
        break;
    case STATE_CFG_AXSGN:
        if(i2c_done){
            next_state = STATE_DELAY;
            delay = 10;
            delay_next_state = STATE_CFG_MODE;
        }
        break;
    case STATE_CFG_MODE:
        if(i2c_done){
            next_state = STATE_DELAY;
            delay = 20;
            delay_next_state = STATE_READ_GRAV;
        }
        break;
    case STATE_READ_GRAV:
        if(i2c_done){
            next_state = STATE_READ_QUAT;
        }
        break;
    case STATE_READ_QUAT:
        if(i2c_done){
            if(reconfig){
                next_state = STATE_RECONFIG;
                reconfig = false;
            }else{
                next_state = STATE_IDLE;
                delay = 10;
            }
        }
        break;
    case STATE_IDLE:
        if(reconfig){
            next_state = STATE_RECONFIG;
            reconfig = false;
        }else if(delay_done){
            next_state = STATE_READ_GRAV;
        }
        break;
    case STATE_RECONFIG:
        if(i2c_done){
            next_state = STATE_DELAY;
            delay = 30;
            delay_next_state = STATE_CFG_AXRMP;
        }
        break;
    }

    // New state actions
    if(next_state != STATE_NONE){
        switch(next_state){
        case STATE_CFG_START:
            // Set to CFG mode
            curr_trans.write_buf[0] = BNO055_OPR_MODE_ADDR;
            curr_trans.write_buf[1] = OPMODE_CFG;
            curr_trans.write_count = 2;
            curr_trans.read_count = 0;
            i2c0_perform(&curr_trans);
            break;
        case STATE_CFG_RST:
            // Reset IMU
            curr_trans.write_buf[0] = BNO055_SYS_TRIGGER_ADDR;
            curr_trans.write_buf[1] = 0x20;
            curr_trans.write_count = 2;
            curr_trans.read_count = 0;
            i2c0_perform(&curr_trans);
            break;
        case STATE_DELAY:
            // Start delay
            timers_bbo055_delay(delay);
            break;
        case STATE_CFG_ID:
            // Verify device ID to determine when reset done
            curr_trans.write_buf[0] = BNO055_CHIP_ID_ADDR;
            curr_trans.write_count = 1;
            curr_trans.read_count = 1;
            i2c0_perform(&curr_trans);
            break;
        case STATE_CFG_PWR:
            // Set power mode
            curr_trans.write_buf[0] = BNO055_PWR_MODE_ADDR;
            curr_trans.write_buf[1] = 0x00; // Normal power mode
            curr_trans.write_count = 2;
            curr_trans.read_count = 0;
            i2c0_perform(&curr_trans);
            break;
        case STATE_CFG_PAGE:
            // Set page reg
            curr_trans.write_buf[0] = BNO055_PAGE_ID_ADDR;
            curr_trans.write_buf[1] = 0x00;
            curr_trans.write_count = 2;
            curr_trans.read_count = 0;
            i2c0_perform(&curr_trans);
            break;
        case STATE_CFG_AXRMP:
            // TODO: Set axis remap reg (i2c write)
            break;
        case STATE_CFG_AXSGN:
            // TODO: Set axis sign reg (i2c write)
            break;
        case STATE_CFG_MODE:
            // TODO: Set in IMU mode (i2c write)
            break;
        case STATE_READ_GRAV:
            // TODO: Read gravity vector (i2c read)
            break;
        case STATE_READ_QUAT:
            // TODO: Read quaternion orientation (i2c read)
            break;
        case STATE_IDLE:
            // Start idle time
            timers_bbo055_delay(delay);
            break;
        case STATE_RECONFIG:
            // TODO: Set to CFG mode (i2c write)
            break;
        }

        // Change current state
        curr_state = next_state;
    }
}

bool bno055_init(void){
    // Initial axis config
    axis_config.x = BNO055_AXIS_X;
    axis_config.y = BNO055_AXIS_Y;
    axis_config.z = BNO055_AXIS_Z;
    axis_config.sx = BNO055_AXIS_POS;
    axis_config.sy = BNO055_AXIS_POS;
    axis_config.sz = BNO055_AXIS_POS;

    // Setup curr_trans
    curr_trans.address = BNO055_ADDR;
    curr_trans.read_buf = read_buf;
    curr_trans.write_buf = write_buf;

    // Attempt an empty transaction to verify the device exists
    // Block until transaction finishes
    curr_trans.write_count = 0;
    curr_trans.read_count = 0;
    i2c0_perform(&curr_trans);
    while(curr_trans.status == I2C_STATUS_BUSY) delay_ms(5);
    if(curr_trans.status == I2C_STATUS_ERROR)
        return false;
    
    // Read the chip ID to verify this is the correct device
    // Block until read finishes
    curr_trans.write_count = 0;
    curr_trans.read_count = 1;
    i2c0_perform(&curr_trans);
    while(curr_trans.status == I2C_STATUS_BUSY) delay_ms(5);
    if(curr_trans.status == I2C_STATUS_ERROR)
        return false;
    if(curr_trans.read_buf[0] != BNO055_ID)
        return false;

    // Move to first state
    curr_state = STATE_NONE;
    bno055_state_machine(false, false);

    // Chip "initialized"
    // There is still asynchronous configuration to be done by the state machine
    return true;
}

void bno055_checki2c(void){
    // The current transaction is still in progress, so do nothing
    if(curr_trans.status == I2C_STATUS_BUSY)
        return;
    
    // Handle state transitions due to i2c finishing
    bno055_state_machine(true, false);
}

void bno055_delay_done(void){
    bno055_state_machine(false, true);
}

void bno055_reconfig(bno055_axis_config new_axis_config){
    axis_config = new_axis_config;
    reconfig = true;
    bno055_state_machine(false, false);
}

bno055_data bno055_get(void){
    return data;
}
