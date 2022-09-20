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
        }
        return;
    }
    
    if(data_startswith(msg, len, (uint8_t[]){'M', 'O', 'D', 'E'}, 4)){
        // MODE set command
        // M,O,D,E,[MODE]
        // MODE: R = RAW, L = LOCAL
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
        // Each speed is a 32-bit float (little endian)

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
        motor_control_feed_watchdog();
    }
}

void cmdctrl_motors_killed(void){
    // Send message telling the computer that the watchdog killed motors
    pccomm_write_msg((uint8_t[]){'W', 'D', 'G', 'K'}, 4);
}
