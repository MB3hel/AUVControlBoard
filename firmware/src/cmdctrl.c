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
#include <dotstar.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned int cmdctrl_mode;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void cmdctrl_init(void){
    cmdctrl_mode = CMDCTRL_MODE_RAW;                // Default to RAW control mode
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
    }
}

void cmdctrl_motors_killed(void){
    // Send message telling the computer that the watchdog killed motors
    pccomm_write_msg((uint8_t[]){'W', 'D', 'G', 'K'}, 4);
}

uint8_t cmdctrl_get_mode(void){
    return cmdctrl_mode;
}

void cmdctrl_send_bno055(void){
    // bno055_data data = bno055_get();
    // TODO: Actually send the data
}
