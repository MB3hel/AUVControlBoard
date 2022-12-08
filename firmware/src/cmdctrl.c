#include <cmdctrl.h>
#include <led.h>
#include <pccomm.h>
#include <conversions.h>
#include <thruster.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Control modes
#define MODE_RAW        0
#define MODE_LOCAL      1
#define MODE_GLOBAL     2
#define MODE_SASSIST    3

// LED colors for different modes
#define COLOR_RAW           100, 100, 0
#define COLOR_LOCAL         10, 0, 100
#define COLOR_GLOBAL        150, 50, 0
#define COLOR_SASSIST       0, 100, 100

// Acknowledge message error codes
#define ACK_ERR_NONE                    0
#define ACK_ERR_UNKNOWN_MSG             1
#define ACK_ERR_INVALID_ARGS            2

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static unsigned int mode;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// CMDCTRL functions / implementatoin
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void cmdctrl_init(void){
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
    data[3] = error_code;
    pccomm_write(data, sizeof(data));
}

void cmdctrl_handle_message(){
    // Helper macros
    #define msg     pccomm_read_buf
    #define len     pccomm_read_len
    #define msg_id  pccomm_read_crc
    #define MSG_STARTS_WITH(x)      data_startswith(msg, len, (x), sizeof((x)))
    #define MSG_EQUALS(x)           data_matches(msg, len, x, sizeof(x))

    
    
    if(MSG_STARTS_WITH(((uint8_t[]){'R', 'A', 'W'}))){
        // RAW speed set command
        // R, A, W, [speed_0], [speed_1], [speed_2], [speed_3], [speed_4], [speed_5], [speed_6], [speed_7]
        // [speed_i] is a 32-bit float

        if(len != 35){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS);
        }else{
            // Message is correct size. Handle it.

            led_set(0, 100, 0);

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
            // TODO: Replace this with calls to a motor_control layer
            thruster_set(speeds);
        }
    }else{
        cmdctrl_acknowledge(msg_id, ACK_ERR_UNKNOWN_MSG);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

