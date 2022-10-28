/**
 * @file cmdctrl.c
 * @author Marcus Behel
 */

#include <cmdctrl.h>
#include <conversions.h>
#include <util.h>
#include <timers.h>
#include <dotstar.h>
#include <usb.h>
#include <motor_control.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const static uint8_t MSG_SET_MODE_PFX[] = {'M', 'O', 'D', 'E'};
const static uint8_t MSG_SET_RAW_PFX[] = {'R', 'A', 'W'};

const static uint8_t MSG_GET_MODE_CMD[] = {'?', 'M', 'O', 'D', 'E'};

const static uint8_t MSG_FEED_MWDT_CMD[] = {'W', 'D', 'G', 'F'};

static unsigned int mode;

// Cached global mode motion target
// static float global_x, global_y, global_z, global_pitch, global_roll, global_yaw;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void cmdctrl_init(void){
    // Zero cached target initially
    // global_x = 0.0f;
    // global_y = 0.0f;
    // global_z = 0.0f;
    // global_pitch = 0.0f;
    // global_roll = 0.0f;
    // global_yaw = 0.0f;

    // Default to RAW control mode
    mode = CMDCTRL_MODE_RAW;
}

void cmdctrl_handle_msg(uint8_t *msg, uint32_t len){
    #define MSG_STARTS_WITH(x)      data_startswith(msg, len, x, sizeof(x))
    #define MSG_EQUALS(x)           data_matches(msg, len, x, sizeof(x))

    // TODO: Thruster inversions, local mode set, global mode set, thruster inversion query, reset command

    if(MSG_STARTS_WITH(MSG_SET_MODE_PFX)){
        // MODE set command
        // M,O,D,E,[MODE]
        // MODE: R = RAW, L = LOCAL, G = GLOBAL

        // Make sure there is enough data
        if(len < 5)
            return;

        // Get mode from message
        switch(msg[4]){
        case 'R':
            mode = CMDCTRL_MODE_RAW;
            break;
        case 'L':
            mode = CMDCTRL_MODE_LOCAL;
            break;
        case 'G':
            mode = CMDCTRL_MODE_GLOBAL;
            break;
        }
    }else if(MSG_STARTS_WITH(MSG_SET_RAW_PFX) && mode == CMDCTRL_MODE_RAW){
        // RAW Mode Speed set command
        // R,A,W,[speed1],[speed2],[speed3],[speed4],[speed5],[speed6],[speed7],[speed8]
        // Each speed is a 32-bit float (little endian) from -1.0 to 1.0

        // Make sure there is enough data
        if(len < 35)
            return;

        // Get speeds from message        
        float speeds[8];
        speeds[0] = conversions_data_to_float(&msg[3], true);
        speeds[1] = conversions_data_to_float(&msg[7], true);
        speeds[2] = conversions_data_to_float(&msg[11], true);
        speeds[3] = conversions_data_to_float(&msg[15], true);
        speeds[4] = conversions_data_to_float(&msg[19], true);
        speeds[5] = conversions_data_to_float(&msg[23], true);
        speeds[6] = conversions_data_to_float(&msg[27], true);
        speeds[7] = conversions_data_to_float(&msg[31], true);

        motor_control_raw(speeds[0], speeds[1], speeds[2], speeds[3], speeds[4],
                speeds[5], speeds[6], speeds[7]);
    }else if(MSG_EQUALS(MSG_GET_MODE_CMD)){
        // Query mode
        uint8_t *response = (uint8_t[]){'M', 'O', 'D', 'E', '\0'};
        switch(mode){
        case CMDCTRL_MODE_RAW:
            response[4] = 'R';
            break;
        case CMDCTRL_MODE_LOCAL:
            response[4] = 'L';
            break;
        case CMDCTRL_MODE_GLOBAL:
            response[4] = 'G';
            break;
        }
        usb_writemsg(response, 5);
    }else if(MSG_EQUALS(MSG_FEED_MWDT_CMD)){
        // Motor watchdog feed command
        // W,D,G,F
        motor_control_watchdog_feed();
    }
}

void cmdctrl_update_led(void){
    switch(mode){
    case CMDCTRL_MODE_RAW:
        dotstar_set(100, 100, 0);
        break;
    case CMDCTRL_MODE_LOCAL:
        dotstar_set(10, 0, 100);
        break;
    case CMDCTRL_MODE_GLOBAL:
        dotstar_set(120, 50, 0);
        break;
    }
}

void cmdctrl_send_sensors(void){
    // uint8_t msg[17];
    // bno055_data imu_dat = bno055_get();

    // // Send IMU orientation (quaternion)
    // msg[0] = 'E';
    // msg[1] = 'U';
    // msg[2] = 'L';
    // msg[3] = 'E';
    // msg[4] = 'R';
    // conversions_float_to_data(imu_dat.euler_pitch, &msg[5], true);
    // conversions_float_to_data(imu_dat.euler_roll, &msg[9], true);
    // conversions_float_to_data(imu_dat.euler_yaw, &msg[13], true);
    // pccomm_write_msg(msg, 17);

    // // Send IMU gravity vector
    // msg[0] = 'G';
    // msg[1] = 'V';
    // msg[2] = 'E';
    // msg[3] = 'C';
    // conversions_float_to_data(imu_dat.grav_x, &msg[4], true);
    // conversions_float_to_data(imu_dat.grav_y, &msg[8], true);
    // conversions_float_to_data(imu_dat.grav_z, &msg[12], true);
    // pccomm_write_msg(msg, 16);
}

void cmdctrl_update_motors(void){
    // bno055_data imu_dat;

    // // Some modes use sensors. In these modes, periodically recalculate
    // // using latest sensor data and a cached speed
    // switch(cmdctrl_mode){
    // case CMDCTRL_MODE_GLOBAL:
    //     // Get sensor data
    //     imu_dat = bno055_get();

    //     // Update motor speeds
    //     motor_control_global(global_x, global_y, global_z, global_pitch, global_roll, 
    //             global_yaw, imu_dat.grav_x, imu_dat.grav_y, imu_dat.grav_z);
    //     break;
    // }
}
