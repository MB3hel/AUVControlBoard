/**
 * @file circular_buffer.c
 * @author Marcus Behel
 */

#include <bno055.h>
#include <util.h>
#include <i2c0.h>
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


// Buffer settings
#define WRITE_BUF_SIZE          4           // max number of write bytes per transaction
#define READ_BUF_SIZE           4           // max number of read bytes per transaction


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t write_buf[WRITE_BUF_SIZE];
uint8_t read_buf[READ_BUF_SIZE];
static i2c_trans trans;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void bno055_state_machine(void){
    // TODO
}

bool bno055_init(void){
    // Setup transaction
    trans.address = BNO055_ADDR;
    trans.write_buf = write_buf;
    trans.read_buf = read_buf;
    
    // Read a byte to verify a device exists with the correct address
    // Blocks until finished
    trans.write_count = 0;
    trans.read_count = 1;
    i2c0_perform(&trans);
    if(trans.status == I2C_STATUS_ERROR)
        return false;
    
    // Delay between transactions or the chip won't respond
    timers_safe_delay(10);

    // Read chip ID register to ensure this is the correct device
    // Blocks until finished
    trans.write_buf[0] = BNO055_CHIP_ID_ADDR;
    trans.write_count = 1;
    trans.read_count = 1;
    i2c0_perform(&trans);
    if(trans.status == I2C_STATUS_ERROR)
        return false;
    if(read_buf[0] != BNO055_ID)
        return false;
    
    // Init done. Correct device is connected.
    return true;
}

void bno055_delay_done(void){

}

void bno055_check_i2c(void){

}

