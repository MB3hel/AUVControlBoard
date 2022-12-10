#include <cmdctrl.h>
#include <led.h>
#include <pccomm.h>
#include <conversions.h>
#include <thruster.h>
#include <motor_control.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Control modes
#define MODE_RAW        0
#define MODE_LOCAL      1
#define MODE_GLOBAL     2
#define MODE_SASSIST    3

// LED colors for different modes
// Different colors on different versions b/c LEDs are different
// Designed to look visually similar
#if defined(CONTROL_BOARD_V1)
#define COLOR_RAW           100, 100, 0
#define COLOR_LOCAL         10, 0, 100
#define COLOR_GLOBAL        150, 50, 0
#define COLOR_SASSIST       0, 100, 100
#elif defined(CONTROL_BOARD_V2)
#define COLOR_RAW           170, 40, 0
#define COLOR_LOCAL         30, 0, 100
#define COLOR_GLOBAL        255, 15, 0
#define COLOR_SASSIST       0, 100, 100
#endif

// Acknowledge message error codes
#define ACK_ERR_NONE                    0   // No error
#define ACK_ERR_UNKNOWN_MSG             1   // The message is not recognized
#define ACK_ERR_INVALID_ARGS            2   // Arguments are invalid (too many, too few, etc)
#define ACK_ERR_INVALID_CMD             3   // Command is known, but invalid at this time
// Note 255 reserved for timeout error codes

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static unsigned int mode;

// Last used raw mode target
static float raw_target[8];

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// CMDCTRL functions / implementation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void cmdctrl_init(void){
    // Initialize targets for all modes to result in no motion

    // Raw mode (zero all motor speeds)
    for(unsigned int i = 0; i < 8; ++i)
        raw_target[i] = 0.0f;

    // TODO: Local mode (zero all)

    // TODO: Global mode (zero all)

    // TODO: Sassist mode (set valid bool to false)

    // Default to raw mode
    mode = MODE_RAW;
    led_set(COLOR_RAW);
}

/**
 * Check if two byte arrays are identical
 * @param a First byte array
 * @param len_a Length of first array
 * @param b Second byte array
 * @param len_b Length of second array
 * @return true If arrays match
 * @return false If arrays do not match
 */
bool data_matches(const uint8_t *a, uint32_t len_a, const uint8_t *b, uint32_t len_b){
    if(len_a != len_b)
        return false;
    for(uint32_t i = 0; i < len_a; ++i){
        if(a[i] != b[i])
            return false;
    }
    return true;
}

/**
 * Check if one array starts with the data in another array
 * @param a The array to search in ("full" data)
 * @param len_a Length of array a
 * @param b The array to search for ("sub" / "prefix" data)
 * @param len_b Length of array b
 * @return true If array a starts with array b
 * @return false If array a does not start with array b
 */
bool data_startswith(const uint8_t *a, uint32_t len_a, const uint8_t *b, uint32_t len_b){
    if(len_a < len_b)
        return false;
    for(uint32_t i = 0; i < len_b; ++i){
        if(a[i] != b[i])
            return false;
    }
    return true;
}

/**
 * Apply the last saved speed for the current mode
 */
void cmdctrl_apply_saved_speed(void){
    switch (mode){
    case MODE_RAW:
        mc_set_raw(raw_target);
        break;
    }
}

/**
 * Acknowledge receipt of a message
 * @param msg_id The ID of the message being acknowledged
 * @param error_code Error code for the acknowledge operation
 */
static void cmdctrl_acknowledge(uint16_t msg_id, uint8_t error_code){
    // A, C, K, [message_id], [error_code]
    // [message_id] is a 16-bit number big endian
    uint8_t data[6];
    data[0] = 'A';
    data[1] = 'C';
    data[2] = 'K';
    conversions_int16_to_data(msg_id, &data[3], false);
    data[5] = error_code;
    pccomm_write(data, sizeof(data));
}

void cmdctrl_handle_message(){
    // Helper macros
    // First two bytes of message are message ID (so skip them for actual message content)
    // Also, skip last two bytes (these are message CRC)
    #define msg     (&pccomm_read_buf[2])
    #define len     (pccomm_read_len - 4)
    
    // Compare message to byte arrays
    #define MSG_STARTS_WITH(x)      data_startswith(msg, len, (x), sizeof((x)))
    #define MSG_EQUALS(x)           data_matches(msg, len, x, sizeof(x))

    // msg_id is first two bytes (unsigned 16-bit int big endian)
    uint16_t msg_id = conversions_data_to_int16(pccomm_read_buf, false);
    
    if(MSG_STARTS_WITH(((uint8_t[]){'R', 'A', 'W'}))){
        // RAW speed set command
        // R, A, W, [speed_0], [speed_1], [speed_2], [speed_3], [speed_4], [speed_5], [speed_6], [speed_7]
        // [speed_i] is a 32-bit float

        if(len != 35){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS);
        }else{
            // Message is correct size. Handle it.

            // Get speeds from message
            raw_target[0] = conversions_data_to_float(&msg[3], true);
            raw_target[1] = conversions_data_to_float(&msg[7], true);
            raw_target[2] = conversions_data_to_float(&msg[11], true);
            raw_target[3] = conversions_data_to_float(&msg[15], true);
            raw_target[4] = conversions_data_to_float(&msg[19], true);
            raw_target[5] = conversions_data_to_float(&msg[23], true);
            raw_target[6] = conversions_data_to_float(&msg[27], true);
            raw_target[7] = conversions_data_to_float(&msg[31], true);

            // Ensure speeds are in valid range
            for(unsigned int i = 0; i < 8; ++i){
                if(raw_target[i] < -1.0f) raw_target[i] = -1.0f;
                else if(raw_target[i] > 1.0f) raw_target[i] = 1.0f;
            }

            // Update mode variable and LED color (if needed)
            if(mode != MODE_RAW){
                mode = MODE_RAW;
                led_set(COLOR_RAW);
            }

            // Feed watchdog when speeds are set
            // Important to call before speed set function in case currently killed
            mc_wdog_feed();

            // Update motor speeds
            mc_set_raw(raw_target);

            // Acknowledge message w/ no error.
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE);
        }
    }else if(MSG_STARTS_WITH(((uint8_t[]){'T', 'I', 'N', 'V'}))){
        // Thruster inversion set command
        // T, I, N, V, [inv]
        // [inv] is an 8-bit int where MSB corresponds to thruster 8 and LSB thruster 1
        //       1 = inverted. 0 = not inverted
        
        if(len != 5){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS);
        }else{
            // Message is correct size. Handle it.
            uint8_t inv = msg[4];
            for(unsigned int i = 0; i < 8; ++i){
                mc_invert[i] = inv & 1;
                inv >>= 1;
            }

            // Reapply saved speed when inversions change
            cmdctrl_apply_saved_speed();

            // Acknowledge message w/ no error.
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE);
        }
    }else if(MSG_EQUALS(((uint8_t[]){'W', 'D', 'G', 'F'}))){
        // Feed motor watchdog command
        // W, D, G, F
        
        // Feed watchdog (as requested)
        mc_wdog_feed();

        // Acknowledge message w/ no error.
        cmdctrl_acknowledge(msg_id, ACK_ERR_NONE);
    }
    else{
        // This is an unrecognized message
        cmdctrl_acknowledge(msg_id, ACK_ERR_UNKNOWN_MSG);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

