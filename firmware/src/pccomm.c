/**
 * @file pccomm.c
 * @author Marcus Behel
 */


#include <pccomm.h>
#include <atmel_start.h>
#include <circular_buffer.h>
#include <cmdctrl.h>
#include <stdlib.h>
#include <util.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MSG_QUEUE_COUNT             6                       // Max number of messages in message queue
#define WRITE_BUF_LEN               512                     // Size of write buffer

#if CONF_USBD_HS_SP
#define RAW_BUF_LEN                 CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ_HS
#else
#define RAW_BUF_LEN                 CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ
#endif


#define START_BYTE                  253                     // Communication  protocol start byte
#define END_BYTE                    254                     // Communication protocol end byte
#define ESCAPE_BYTE                 255                     // Communication protocol end byte

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static bool parse_started, parse_escaped;                           // Used to identify complete message

static bool initialized = false;                                    // Tracks if initialized

static uint8_t buf_raw_rx[RAW_BUF_LEN];                             // Used by acm_read to read data into
static uint8_t buf_raw_tx[RAW_BUF_LEN];                             // Used by acm_write to write data from

static uint8_t curr_msg[PCCOMM_MAX_MSG_LEN + 2];                    // Holds the message currently being received & crc
static uint32_t curr_msg_pos;                                       // Current size of current message

static uint8_t buf_write_arr[WRITE_BUF_LEN];                        // Backing array for write circular buffer
static circular_buffer buf_write;                                   // Holds data waiting to be written

static uint8_t msg_queue[MSG_QUEUE_COUNT][PCCOMM_MAX_MSG_LEN];      // Holds unprocessed received messages
static uint32_t msg_queue_pos[MSG_QUEUE_COUNT];                     // Size of messages in each spot in the queue
static uint32_t msg_queue_widx;                                     // Index in queue to place next message at
static uint32_t msg_queue_ridx;                                     // Index in queue to read next message from


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Callback forward declarations
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static bool cb_usb_state(usb_cdc_control_signal_t state);
static bool cb_usb_read(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count);
static bool cb_usb_write(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count);


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool pccomm_init(void){
    if(initialized)
        return true;

    // Can't register state callback until enabled
    if(!cdcdf_acm_is_enabled())
        return false;
    cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)cb_usb_state);

    // Other initialization
    cb_init(&buf_write, buf_write_arr, WRITE_BUF_LEN);
    parse_started = false;
    parse_escaped = false;
    curr_msg_pos = 0;
    msg_queue_widx = 0;
    msg_queue_ridx = 0;
    for(uint32_t i = 0; i < MSG_QUEUE_COUNT; ++i){
        msg_queue_pos[i] = 0;
    }
    initialized = true;
    return true;
}

void pccomm_write_msg(uint8_t *data, uint32_t len){
    cb_write(&buf_write, START_BYTE);
    for(uint8_t i = 0; i < len; ++i){
        if(data[i] == START_BYTE || data[i] == END_BYTE || data[i] == ESCAPE_BYTE)
            cb_write(&buf_write, ESCAPE_BYTE);
        cb_write(&buf_write, data[i]);
    }

    // Calculate and add crc
    uint16_t crc = crc16_ccitt(data, len);
    cb_write(&buf_write, (crc >> 8) & 0xFF);
    cb_write(&buf_write, crc & 0xFF);

    cb_write(&buf_write, END_BYTE);

    // Triggers a write callback if not currently transmitting
    // Write callback then pulls data out of queue and transmits it
    // If currently transmitting, returns an error code (ignored)
    // This is ok since callback will be called when current transmit finishes
    cdcdf_acm_write((uint8_t *)buf_raw_tx, 0);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Callbacks
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static bool cb_usb_state(usb_cdc_control_signal_t state){
    if (state.rs232.DTR) {
        // Register callbacks **AFTER** endpoints allocated
        cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, (FUNC_PTR)cb_usb_read);
        cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, (FUNC_PTR)cb_usb_write);

        // Initial read
        // Any data read will be available in the read callback
        // The next read should occur in the read callback
        cdcdf_acm_read((uint8_t *)buf_raw_rx, RAW_BUF_LEN);
    }

    // No error
    return false;
}

// Called when read completes. "count" is number of bytes that were read
static bool cb_usb_read(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count){
    // Parse the read data and add to current message if warranted
    for(uint32_t i = 0; i < count; ++i){
        if(parse_escaped){
            // Ignore invalid escaped data
            if(buf_raw_rx[i] == START_BYTE || buf_raw_rx[i] == END_BYTE || buf_raw_rx[i] == ESCAPE_BYTE)
                curr_msg[curr_msg_pos++] = buf_raw_rx[i];
            parse_escaped = false;
        }else{
            switch(buf_raw_rx[i]){
            case START_BYTE:
                if(parse_started){
                    // Got second start byte.
                    // Discard previous data (never got end byte)
                    curr_msg_pos = 0;
                }
                parse_started = true;
                break;
            case END_BYTE:
                if(!parse_started)
                    break;

                // curr_msg is now complete
                // Move it to the next spot in msg_queue
                // Ignore empty messages
                if(curr_msg_pos > 0){
                    // Only queue message if crc is valid
                    uint16_t read_crc = (curr_msg[curr_msg_pos - 2] << 8) | curr_msg[curr_msg_pos - 1];
                    uint16_t calc_crc = crc16_ccitt(curr_msg, curr_msg_pos - 2);
                    if(read_crc == calc_crc){
                        memcpy(msg_queue[msg_queue_widx], curr_msg, curr_msg_pos - 2);
                        msg_queue_pos[msg_queue_widx] = curr_msg_pos - 2;
                        msg_queue_widx++;
                        if(msg_queue_widx >= MSG_QUEUE_COUNT)
                            msg_queue_widx = 0;
                        parse_started = false;
                    }
                }

                curr_msg_pos = 0;
                break;
            case ESCAPE_BYTE:
                if(!parse_started)
                    break;
                parse_escaped = true;
                break;
            default:
                if(!parse_started)
                    break;
                curr_msg[curr_msg_pos++] = buf_raw_rx[i];
            }
        }
    }

    // Start next read
    cdcdf_acm_read((uint8_t*)buf_raw_rx, RAW_BUF_LEN);

    // No error
    return false;
}

// Called when write complets. "count" is number of bytes that were just written
static bool cb_usb_write(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count){
    // Move data from write buffer into raw tx buffer
    uint32_t i;
    for(i = 0; i < RAW_BUF_LEN; ++i){
        if(CB_EMPTY(&buf_write))
            break;  // Nothing else to transmit
        cb_read(&buf_write, &buf_raw_tx[i]);
    }

    // Start next write (if anything to write)
    if(i != 0){
        cdcdf_acm_write((uint8_t*)buf_raw_tx, i);
    }
    
    // No error
    return false;
}

uint32_t pccomm_get_msg(uint8_t *dest){
    if(msg_queue_pos[msg_queue_ridx] == 0){
        // Next message to read is of size 0
        // This means there are no messages in the queue
        // Thus do not increment read index
        return 0;
    }else{
        memcpy(dest, msg_queue[msg_queue_ridx], msg_queue_pos[msg_queue_ridx]);     // Copy data to dest
        uint32_t tmp = msg_queue_pos[msg_queue_ridx];                               // Store size of copied data for later
        msg_queue_pos[msg_queue_ridx] = 0;                                          // Indicate this slot is empty
        msg_queue_ridx++;                                                           // Move to next slot in queue
        if(msg_queue_ridx >= MSG_QUEUE_COUNT)                                       // Rollover if needed
            msg_queue_ridx = 0;
        return tmp;                                                                 // Return number of bytes copied
    }
}
