/**
 * @file cmdctrl.c
 * @author Marcus Behel
 */

#include <cmdctrl.h>
#include <util.h>
#include <pccomm.h>
#include <conversions.h>
#include <stdbool.h>
#include <motor_control.h>
#include <stdint.h>
#include <stdlib.h>
#include <bno055.h>
#include <timers.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static unsigned int cmdctrl_mode;

// Cached global mode motion target
static float global_x, global_y, global_z, global_pitch, global_roll, global_yaw;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void cmdctrl_init(void){
    // Zero cached target initially
    global_x = 0.0f;
    global_y = 0.0f;
    global_z = 0.0f;
    global_pitch = 0.0f;
    global_roll = 0.0f;
    global_yaw = 0.0f;

    // Default to RAW control mode
    cmdctrl_mode = CMDCTRL_MODE_RAW;
}

void cmdctrl_handle_msg(uint8_t *msg, uint32_t len){
    if(data_startswith(msg, len, (uint8_t[]){'?'}, 1)){
        // Handle queries
        if(data_matches(&msg[1], len - 1, (uint8_t[]){'M', 'O', 'D', 'E'}, 4)){
            // Query mode
            uint8_t *response = (uint8_t[]){'M', 'O', 'D', 'E', '\0'};
            switch(cmdctrl_mode){
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
            pccomm_write_msg(response, 5);
        }else if(data_matches(&msg[1], len -1, (uint8_t[]){'T', 'I', 'N', 'V'}, 4)){
            // Query thruster inversions
            uint8_t *response = (uint8_t[]){'T', 'I', 'N', 'V', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'};
            for(size_t i = 0; i < 8; ++i){
                response[4 + i] = motor_control_tinv[i];
            }
            pccomm_write_msg(response, 12);
        }
        return;
    }
    
    if(data_startswith(msg, len, (uint8_t[]){'M', 'O', 'D', 'E'}, 4)){
        // MODE set command
        // M,O,D,E,[MODE]
        // MODE: R = RAW, L = LOCAL

        // Make sure there is enough data
        if(len < 5)
            return;

        // Get mode from message
        switch(msg[4]){
        case 'R':
            cmdctrl_mode = CMDCTRL_MODE_RAW;
            break;
        case 'L':
            cmdctrl_mode = CMDCTRL_MODE_LOCAL;
            break;
        case 'G':
            cmdctrl_mode = CMDCTRL_MODE_GLOBAL;
            global_x = 0.0f;
            global_y = 0.0f;
            global_z = 0.0f;
            global_pitch = 0.0f;
            global_roll = 0.0f;
            global_yaw = 0.0f;
            break;
        }
    }else if(data_startswith(msg, len, (uint8_t[]){'R', 'A', 'W'}, 3) && cmdctrl_mode == CMDCTRL_MODE_RAW){
        // RAW speed set command (only works in RAW mode)
        // R,A,W,[speed1],[speed2],...,[speed8]
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

        // Update motor speeds
        motor_control_raw(speeds[0], speeds[1], speeds[2], speeds[3], 
                speeds[4], speeds[5], speeds[6], speeds[7]);
    }else if(data_matches(msg, len, (uint8_t[]){'W', 'D', 'G', 'F'}, 4)){
        // Motor watchdog feed command
        // W,D,G,F
        motor_control_watchdog_feed();
    }else if(data_startswith(msg, len, (uint8_t[]){'T', 'I', 'N', 'V'}, 4)){
        // Set thruster inversion command
        // TINV[i1][i2][i3][i4][i5][i6][i7][i8]
        // Each "i" is a 1 or 0 where 1 = inverted, 0 = not inverted
        // Ensure enough data
        if(len < 12)
            return;
        
        // Parse inversions
        for(size_t i = 0; i < 8; ++i){
            motor_control_tinv[i] = msg[i + 4];
        }
    }else if(data_startswith(msg, len, (uint8_t[]){'L', 'O', 'C', 'A', 'L'}, 5) && cmdctrl_mode == CMDCTRL_MODE_LOCAL){
        // LOCAL speed set command (only works in LOCAL mode)
        // L,O,C,A,L,[x],[y],[z],[pitch],[roll],[yaw]
        // Each speed (x, y, z, pitch, roll, yaw) is a float (32-bit) from -1.0 to 1.0
        float x, y, z, pitch, roll, yaw;

        // Ensure enough data
        if(len < 29)
            return;

        // Get speeds from message
        x = conversions_data_to_float(&msg[5], true);
        y = conversions_data_to_float(&msg[9], true);
        z = conversions_data_to_float(&msg[13], true);
        pitch = conversions_data_to_float(&msg[17], true);
        roll = conversions_data_to_float(&msg[21], true);
        yaw = conversions_data_to_float(&msg[25], true);

        // Update motor speeds
        motor_control_local(x, y, z, pitch, roll, yaw);
    }else if(data_startswith(msg, len, (uint8_t[]){'G', 'L', 'O', 'B', 'A', 'L'}, 6) && cmdctrl_mode == CMDCTRL_MODE_GLOBAL){
        // GLOBAL speed set command (only works in LOCAL mode)
        // G,L,O,B,A,L,[x],[y],[z],[pitch],[roll],[yaw]
        // Each speed (x, y, z, pitch, roll, yaw) is a float (32-bit) from -1.0 to 1.0

        // Ensure enough data
        if(len < 30)
            return;

        // Get speeds from message
        global_x = conversions_data_to_float(&msg[6], true);
        global_y = conversions_data_to_float(&msg[10], true);
        global_z = conversions_data_to_float(&msg[14], true);
        global_pitch = conversions_data_to_float(&msg[18], true);
        global_roll = conversions_data_to_float(&msg[22], true);
        global_yaw = conversions_data_to_float(&msg[26], true);

        // Get sensor data
        bno055_data imu_dat = bno055_get();

        // Update motor speeds
        motor_control_global(global_x, global_y, global_z, global_pitch, global_roll, 
                global_yaw, imu_dat.grav_x, imu_dat.grav_y, imu_dat.grav_z);
    }else if(data_matches(msg, len, (uint8_t[]){'R', 'E', 'S', 'E', 'T'}, 5)){
        // RESET command (reset MCU)
        // R,E,S,E,T
        timers_reset_now();
    }
}

void cmdctrl_motors_killed(void){
    // Clear cached target info
    global_x = 0.0f;
    global_y = 0.0f;
    global_z = 0.0f;
    global_pitch = 0.0f;
    global_roll = 0.0f;
    global_yaw = 0.0f;

    // Send message telling the computer that the watchdog killed motors
    pccomm_write_msg((uint8_t[]){'W', 'D', 'G', 'K'}, 4);
}

uint8_t cmdctrl_get_mode(void){
    return cmdctrl_mode;
}

void cmdctrl_send_sensors(void){
    uint8_t msg[20];
    bno055_data imu_dat = bno055_get();

    // Send IMU orientation (quaternion)
    msg[0] = 'Q';
    msg[1] = 'U';
    msg[2] = 'A';
    msg[3] = 'T';
    conversions_float_to_data(imu_dat.quat_w, &msg[4], true);
    conversions_float_to_data(imu_dat.quat_x, &msg[8], true);
    conversions_float_to_data(imu_dat.quat_y, &msg[12], true);
    conversions_float_to_data(imu_dat.quat_z, &msg[16], true);
    pccomm_write_msg(msg, 20);

    // Send IMU gravity vector
    msg[0] = 'G';
    msg[1] = 'V';
    msg[2] = 'E';
    msg[3] = 'C';
    conversions_float_to_data(imu_dat.grav_x, &msg[4], true);
    conversions_float_to_data(imu_dat.grav_y, &msg[8], true);
    conversions_float_to_data(imu_dat.grav_z, &msg[12], true);
    pccomm_write_msg(msg, 16);
}

void cmdctrl_update_motors(void){
    bno055_data imu_dat;

    // Some modes use sensors. In these modes, periodically recalculate
    // using latest sensor data and a cached speed
    switch(cmdctrl_mode){
    case CMDCTRL_MODE_GLOBAL:
        // Get sensor data
        imu_dat = bno055_get();

        // Update motor speeds
        motor_control_global(global_x, global_y, global_z, global_pitch, global_roll, 
                global_yaw, imu_dat.grav_x, imu_dat.grav_y, imu_dat.grav_z);
        break;
    }
}
