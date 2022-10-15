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
#include <flags.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MSG_QUEUE_COUNT             6                                       // Max number of messages in message queue
#define WRITE_BUF_LEN               512                                     // Size of write buffer
#define READ_BUF_LEN                256                                     // Size of read buffer

#if CONF_USBD_HS_SP
#define RAW_BUF_LEN                 CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ_HS
#else
#define RAW_BUF_LEN                 CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ
#endif


#define START_BYTE                  253                                     // Communication  protocol start byte
#define END_BYTE                    254                                     // Communication protocol end byte
#define ESCAPE_BYTE                 255                                     // Communication protocol end byte

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// State

static bool parse_started, parse_escaped;                                   // Used to identify complete message
static bool initialized = false;                                            // Tracks if initialized


// Receive buffers

// calls to cdcdf_acm_read() will read data into buf_raw_rx
// The read callback is run when there is data in buf_raw_rx
// The callback just copies from buf_raw_rx to buf_read
// The callback should be treated as an ISR and should run fast
// to prevent starvation of the main thread when large volumes of data are sent
// Later, when pccomm_process is called, the data in buf_read is parsed and
// copied into the message queue
// The message queue is an array of byte arrays = an array of messages
// This queue is implemented itself as a ring
// widx points to the message currently being written (by pccomm)
// When a complete message is written widx is incremented (with rollover)
// ridx points to the next complete message in the queue
// This queue is filled with messages by pccomm_process
// Note: if ridx == widx queue is either full or empty
// Calls to pccomm_get_msg remove a message from the queue

static volatile uint8_t buf_raw_rx[RAW_BUF_LEN];                            // Used by acm_read to read data into

static volatile uint8_t buf_read_arr[READ_BUF_LEN];                         // Holds data read by one or more read ops
static volatile circular_buffer buf_read;                                   // Read buffer (circular buffer)

static volatile uint8_t msg_queue[MSG_QUEUE_COUNT][PCCOMM_MAX_MSG_LEN+2];   // Holds complete received messages & crcs
static volatile uint32_t msg_queue_pos[MSG_QUEUE_COUNT];                    // Size of messages in each spot of queue
static volatile uint32_t msg_queue_widx;                                    // Index in queue to place next message at
static volatile uint32_t msg_queue_ridx;                                    // Index in queue to read next message from
static volatile uint32_t msg_queue_count;                                   // Number of complete messages in queue


// Transmit buffers
static volatile uint8_t buf_raw_tx[RAW_BUF_LEN];                            // Used by acm_write to write data from

static volatile uint8_t buf_write_arr[WRITE_BUF_LEN];                       // Backing array for write circular buffer
static volatile circular_buffer buf_write;                                  // Holds data waiting to be written

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
    cb_init(&buf_read, buf_read_arr, READ_BUF_LEN);
    parse_started = false;
    parse_escaped = false;
    msg_queue_widx = 0;
    msg_queue_ridx = 0;
    for(uint32_t i = 0; i < MSG_QUEUE_COUNT; ++i){
        msg_queue_pos[i] = 0;
    }
    msg_queue_count = 0;
    initialized = true;
    return true;
}

void pccomm_process(void){
    uint8_t c;

    // Nowhere to put processed data (msg_queue full)
    // This should never happen...
    // If it does, message queue should probably be larger
    if(msg_queue_count == MSG_QUEUE_COUNT){
        return;
    }

    // Parse finite number of bytes from the read buffer
    // Cannot just parse until read buffer is empty
    // Data could be inserted into read buffer by callback while this runs
    // Thus waiting until buffer is empty could block forever
    for(uint32_t i = 0; i < READ_BUF_LEN; ++i){
        if(CB_EMPTY(&buf_read))
            break;
        
        // Parse byte and copy into msg_queue[widx] as needed
        cb_read(&buf_read, &c);
        if(parse_escaped){
            // Currently escaped (last byte was ESCAPE_BYTE)
            // Copy only valid escaped bytes into buffer
            if(c == START_BYTE || c == END_BYTE || c == ESCAPE_BYTE)
                msg_queue[msg_queue_widx][msg_queue_pos[msg_queue_widx]++] = c;
            
            // byte after ESCAPE_BYTE handled now
            parse_escaped = false;
        }else{
            switch(c){
            case START_BYTE:
                if(parse_started){
                    // Got a second start byte
                    // Discard previous data (never got end byte)
                    msg_queue_pos[msg_queue_widx] = 0;
                }
                parse_started = true;
                break;
            case END_BYTE:
                if(!parse_started)
                    break;
                // Got a complete message
                if(msg_queue_pos[msg_queue_widx] >= 3){
                    // Must have at least one data byte + crc
                    volatile uint8_t *crc_data = &msg_queue[msg_queue_widx][msg_queue_pos[msg_queue_widx] - 2];
                    uint16_t read_crc = (crc_data[0] << 8) | crc_data[1];
                    uint16_t calc_crc = crc16_ccitt(msg_queue[msg_queue_widx], msg_queue_pos[msg_queue_widx] - 2);
                    if(read_crc == calc_crc){
                        // Valid message. Move widx to next spot in queue.
                        msg_queue_widx++;
                        if(msg_queue_widx >= MSG_QUEUE_COUNT)
                            msg_queue_widx = 0;
                        msg_queue_count++;

                        // Indicate to main tree that there is a message it should process
                        FLAG_SET(flags_main, FLAG_MAIN_PCCOMM_MSG);
                    }else{
                        // Not a valid message. Clear queue spot
                        msg_queue_pos[msg_queue_widx] = 0;
                    }
                }else{
                    // Not a valid message. Clear queue spot
                    msg_queue_pos[msg_queue_widx] = 0;
                }
                break;
            case ESCAPE_BYTE:
                if(!parse_started)
                    break;
                parse_escaped = true;
                break;
            default:
                if(!parse_started)
                    break;
                // Add byte to message
                msg_queue[msg_queue_widx][msg_queue_pos[msg_queue_widx]++] = c;
                break;
            }
        }

    }
}

void pccomm_write_msg(uint8_t *data, uint32_t len){
    if(!initialized)
        return;
    
    cb_write(&buf_write, START_BYTE);
    for(uint8_t i = 0; i < len; ++i){
        if(data[i] == START_BYTE || data[i] == END_BYTE || data[i] == ESCAPE_BYTE)
            cb_write(&buf_write, ESCAPE_BYTE);
        cb_write(&buf_write, data[i]);
    }

    // Calculate and add crc
    uint16_t crc = crc16_ccitt(data, len);
    uint8_t high_byte = (crc >> 8) & 0xFF;
    uint8_t low_byte = crc & 0xFF;
    if(high_byte == START_BYTE || high_byte == END_BYTE || high_byte == ESCAPE_BYTE)
        cb_write(&buf_write, ESCAPE_BYTE);
    cb_write(&buf_write, high_byte);
    if(low_byte == START_BYTE || low_byte == END_BYTE || low_byte == ESCAPE_BYTE)
        cb_write(&buf_write, ESCAPE_BYTE);
    cb_write(&buf_write, low_byte);

    cb_write(&buf_write, END_BYTE);

    // Triggers a write callback if not currently transmitting
    // Write callback then pulls data out of queue and transmits it
    // If currently transmitting, returns an error code (ignored)
    // This is ok since callback will be called when current transmit finishes
    cdcdf_acm_write((uint8_t *)buf_raw_tx, 0);
}

uint32_t pccomm_get_msg(uint8_t *dest){
    uint32_t res;
    if(msg_queue_count == 0){
        // Nothing to get
        return 0;
    }

    // Copy next available message
    // Note: Exclude last two bytes because these are crc
    vmemcpy(dest, msg_queue[msg_queue_ridx], msg_queue_pos[msg_queue_ridx] - 2);

    // Clear this spot in queue (so next write will start at index 0)
    res = msg_queue_pos[msg_queue_ridx] - 2;
    msg_queue_pos[msg_queue_ridx] = 0;

    // Increment ridx (rolling over as needed)
    msg_queue_ridx++;
    if(msg_queue_ridx == MSG_QUEUE_COUNT)
        msg_queue_ridx = 0;

    // Decrement counter
    msg_queue_count--;

    if(msg_queue_count != 0){
        // Still message(s) in the queue
        // Indicate to main tree that there is a message it should process
        FLAG_SET(flags_main, FLAG_MAIN_PCCOMM_MSG);
    }

    return res;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Callbacks
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// This is called from an ISR. Treat this function as an ISR.
// System will wake from sleep after this function is called
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
// This is called from an ISR. Treat this function as an ISR.
// System will wake from sleep after this function is called
static bool cb_usb_read(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count){
    // Copy the read data to be handled (parsed) later (when pccomm_process) is called
    for(uint32_t i = 0; i < count; ++i){
        cb_write(&buf_read, buf_raw_rx[i]);
    }

    // Data is in read buffer. Indicate that pccomm_process should be called by main tree
    FLAG_SET(flags_main, FLAG_MAIN_PCCOMM_PROC);

    // Start next read
    cdcdf_acm_read((uint8_t*)buf_raw_rx, RAW_BUF_LEN);

    // No error
    return false;
}

// Called when write completes. "count" is number of bytes that were just written
// This is called from an ISR. Treat this function as an ISR.
// System will wake from sleep after this function is called
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
