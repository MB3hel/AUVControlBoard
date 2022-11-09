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
#include <bno055.h>
#include <ms5837.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const static uint8_t MSG_SET_MODE_PFX[] = {'M', 'O', 'D', 'E'};
const static uint8_t MSG_SET_TINV_PFX[] = {'T', 'I', 'N', 'V'};
const static uint8_t MSG_SET_RAW_PFX[] = {'R', 'A', 'W'};
const static uint8_t MSG_SET_LOCAL_PFX[] = {'L', 'O', 'C', 'A', 'L'};
const static uint8_t MSG_SET_GLOBAL_PFX[] = {'G', 'L', 'O', 'B', 'A', 'L'};

const static uint8_t MSG_GET_MODE_CMD[] = {'?', 'M', 'O', 'D', 'E'};
const static uint8_t MSG_GET_TINV_CMD[] = {'?', 'T', 'I', 'N', 'V'};

const static uint8_t MSG_FEED_MWDT_CMD[] = {'W', 'D', 'G', 'F'};

static unsigned int mode;

// Cached global mode motion target
// Need to store last setting because periodic recalculations are required in case of orientation changes
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

        // Update motor speeds
        motor_control_watchdog_feed();
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
    }else if(MSG_STARTS_WITH(MSG_SET_TINV_PFX)){
        // Set thruster inversion command
        // TINV[i1][i2][i3][i4][i5][i6][i7][i8]
        // Each "i" is a 1 or 0 where 1 = inverted, 0 = not inverted
        // Ensure enough data
        if(len < 12)
            return;
        
        // Parse inversions
        for(uint32_t i = 0; i < 8; ++i){
            motor_control_tinv[i] = msg[i + 4];
        }
    }else if(MSG_EQUALS(MSG_GET_TINV_CMD)){
        // Query thruster inversions
        uint8_t *response = (uint8_t[]){'T', 'I', 'N', 'V', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'};
        for(uint32_t i = 0; i < 8; ++i){
            response[4 + i] = motor_control_tinv[i];
        }
        usb_writemsg(response, 12);
    }else if(MSG_STARTS_WITH(MSG_SET_LOCAL_PFX)){
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
        motor_control_watchdog_feed();
        motor_control_local(x, y, z, pitch, roll, yaw);
    }else if(MSG_STARTS_WITH(MSG_SET_GLOBAL_PFX)){
        // GLOBAL speed set command (only works in LOCAL mode)
        // G,L,O,B,A,L,[x],[y],[z],[pitch],[roll],[yaw]
        // Each speed (x, y, z, pitch, roll, yaw) is a float (32-bit) from -1.0 to 1.0

        // Ensure enough data
        if(len < 30)
            return;

        // GLOBAL mode requires IMU to be connected
        // If no IMU, send error response and do not set in this mode
        if(!bno055_connected()){
            usb_writemsg((uint8_t[]){'E', 'R', 'R', 'I', 'M', 'U'}, 6);
        }else{
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
        }
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
        dotstar_set(150, 50, 0);
        break;
    }
}

void cmdctrl_motors_killed(void){
    // Clear cached states when motors killed
    global_x = 0.0f;
    global_y = 0.0f;
    global_z = 0.0f;
    global_pitch = 0.0f;
    global_roll = 0.0f;
    global_yaw = 0.0f;
    
    // Send message telling the computer that the watchdog killed motors
    usb_writemsg((uint8_t[]){'W', 'D', 'G', 'K'}, 4);
}

void cmdctrl_update_motors(void){
    bno055_data imu_dat;

    // Some modes use sensors. In these modes, periodically recalculate
    // using latest sensor data and a cached speed
    switch(mode){
    case CMDCTRL_MODE_GLOBAL:
        // Get sensor data
        imu_dat = bno055_get();

        // TODO: IMU connected check

        // Update motor speeds
        motor_control_global(global_x, global_y, global_z, global_pitch, global_roll, 
                global_yaw, imu_dat.grav_x, imu_dat.grav_y, imu_dat.grav_z);
        break;
    }
}
