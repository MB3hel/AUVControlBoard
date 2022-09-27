/**
 * @file pccomm.c
 * @author Marcus Behel
 */


#include <pccomm.h>
#include <atmel_start.h>
#include <circular_buffer.h>
#include <cmdctrl.h>
#include <stdlib.h>


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

static uint8_t curr_msg[PCCOMM_MAX_MSG_LEN];                        // Holds the message currently being received
static size_t curr_msg_pos;                                         // Current size of current message

static uint8_t buf_write_arr[WRITE_BUF_LEN];                        // Backing array for write circular buffer
static circular_buffer buf_write;                                   // Holds data waiting to be written

static uint8_t msg_queue[MSG_QUEUE_COUNT][PCCOMM_MAX_MSG_LEN];      // Holds unprocessed received messages


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
    uint16_t crc = pccomm_crc16(data, len);
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
    // TODO

    // No error
    return false;
}

// Called when write complets. "count" is number of bytes that were just written
static bool cb_usb_write(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count){
    // TODO
    
    // No error
    return false;
}
