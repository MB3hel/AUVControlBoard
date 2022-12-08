#include <pccomm.h>
#include <stdint.h>
#include <tusb.h>

// Comm protocol special bytes
#define START_BYTE          253
#define END_BYTE            254
#define ESCAPE_BYTE         255


// Holds the message currently being read (size + 2 because of 2 bytes for CRC16)
static uint8_t read_buf[PCCOMM_MAX_MSG_LEN + 2];
static unsigned int read_len = 0;


bool pccomm_read_and_parse(void){
    // Messages can be read & parsed over multiple calls
    // Thus, need to keep  track of current state and current message
    static bool parse_started = false;
    static bool parse_escaped = false;

    // Read available bytes one at a time and parse them
    uint8_t byte;
    while(tud_cdc_available()){        
        // Read the next available character
        byte = tud_cdc_read_char();

        // If message queue is full, discard bytes
        if(read_len == sizeof(read_buf))
            continue;

        // Parse the meaning of this byte
        if(parse_escaped){
            // Currently escaped (previous byte was ESCAPE_BYTE)
            // Handle **valid** escape sequences (only special bytes can be escaped)
            // Ignore invalid escape sequences
            if(byte == START_BYTE || byte == END_BYTE || byte == ESCAPE_BYTE)
                read_buf[read_len++] = byte;
            
            // Handled the byte after an escape byte. No longer escaped
            parse_escaped = false;
        }else if(parse_started){
            // If a start byte was previously received, handle this byte
            switch(byte){
            case START_BYTE:
                // Handle start byte (special meaning when not escaped)
                // Discard old data when a start byte received
                read_len = 0;
                break;
            case END_BYTE:
                // Handle end byte (special meaning when not escaped)
                // End byte means the buffer now holds the entire message
                return true;
                break;
            case ESCAPE_BYTE:
                // Handle escape byte (special meaning when not escaped)
                parse_escaped = true;
                break;
            default:
                // Handle normal bytes (these are just data)
                read_buf[read_len++] = byte;
                break;
            }
        }else if(byte == START_BYTE){
            // Received a start byte.
            parse_started = true;
        }
    }

    return false;
}
