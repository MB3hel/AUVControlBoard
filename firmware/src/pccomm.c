/**
 * @file pccomm.c
 * @author Marcus Behel (mgbehel@ncsu.edu)
 */


#include <pccomm.h>
#include <atmel_start.h>
#include <circular_buffer.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define PCCOMM_RX_BUF_SZ        64                  // Buffer for receiving data into
#define PCCOMM_TX_BUF_SZ        64                  // Buffer for data currently being transmitted
#define PCCOMM_RECV_MSG_SZ      128                 // Size of buffer to hold a received message
#define PCCOMM_TX_QUEUE_SZ      128                 // Size of buffer to hold data waiting to be transmitted

// Defined by communication protocol
#define PCCOMM_START_BYTE       253
#define PCCOMM_END_BYTE         254
#define PCCOMM_ESCAPE_BYTE      255


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t pccomm_rx_buf[PCCOMM_RX_BUF_SZ];            // Buffer to hold data currently being received

uint8_t pccomm_recv_msg[PCCOMM_RECV_MSG_SZ];        // Buffer to hold a single received message
uint16_t pccomm_recv_msg_pos;                       // Current position in received message buffer

uint8_t pccomm_tx_buf[PCCOMM_TX_BUF_SZ];            // Buffer to hold data currently being transmitted

uint8_t pccomm_tx_queue_arr[PCCOMM_TX_QUEUE_SZ];    // Backing array to hold messages waiting to be sent
circular_buffer pccomm_tx_queue;                    // Circular buffer for tx queue

bool pccomm_parse_escaped;                          // Tracks if data being read is in escaped state
bool pccomm_parse_started;                          // Tracks if data being read is in started state

const uint8_t PCCOMM_MSG_LED_PREFIX[] = {'R', 'L', 'E', 'D'};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Callback forward declarations
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool pccomm_cb_usb_state(usb_cdc_control_signal_t state);
bool pccomm_cb_usb_read(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count);
bool pccomm_cb_usb_write(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count);


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Check if two byte arrays are identical
 * @param a First byte array
 * @param len_a Length of first array
 * @param b Second byte array
 * @param len_b Length of second array
 * @return true If arrays match
 * @return false If arrays do not match
 */
bool pccomm_matches(const uint8_t *a, uint32_t len_a, const uint8_t *b, uint32_t len_b){
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
bool pccomm_startswith(const uint8_t *a, uint32_t len_a, const uint8_t *b, uint32_t len_b){
    if(len_a < len_b)
        return false;
    for(uint32_t i = 0; i < len_b; ++i){
        if(a[i] != b[i])
            return false;
    }
    return true;
}

void pccomm_init(void){
    cb_init(&pccomm_tx_queue, pccomm_tx_queue_arr, PCCOMM_TX_QUEUE_SZ);
    pccomm_recv_msg_pos = 0;
    pccomm_parse_escaped = false;
    pccomm_parse_started = false;
    while(!cdcdf_acm_is_enabled()){
        delay_ms(100);
    }
    cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)pccomm_cb_usb_state);
}

void pccomm_write_msg(uint8_t *data, uint32_t len){
    cb_write(&pccomm_tx_queue, PCCOMM_START_BYTE);
    for(uint8_t i = 0; i < len; ++i){
        if(data[i] == PCCOMM_START_BYTE || data[i] == PCCOMM_END_BYTE || data[i] == PCCOMM_ESCAPE_BYTE)
            cb_write(&pccomm_tx_queue, PCCOMM_ESCAPE_BYTE);
        cb_write(&pccomm_tx_queue, data[i]);
    }

    // Ignore CRC for now
    // TODO: Calculate CRC instead of ignoring
    cb_write(&pccomm_tx_queue, 0x00);
    cb_write(&pccomm_tx_queue, 0x00);

    cb_write(&pccomm_tx_queue, PCCOMM_END_BYTE);

    // Triggers a write callback if not currently transmitting
    // Write callback then pulls data out of queue and transmits it
    // If currently transmitting, returns an error code (ignored)
    // This is ok since callback will be called when current transmit finishes
    cdcdf_acm_write((uint8_t *)pccomm_tx_buf, 0);
}

void pccomm_handle_received_msg(void){
    // Called by the read callback when a complete message is received
    // Message will be in the pccomm_recv_msg buffer with length pccomm_recv_msg_pos
    // The last two bytes of the message will be the CRC
    // The message will not be escaped (meaning it is the original payload)

    // Ignore CRC for now
    // TODO: Verify CRC instead of ignoring
    pccomm_recv_msg_pos -= 2;

    if(pccomm_startswith(pccomm_recv_msg, pccomm_recv_msg_pos, PCCOMM_MSG_LED_PREFIX, sizeof(PCCOMM_MSG_LED_PREFIX))){
        switch(pccomm_recv_msg[sizeof(PCCOMM_MSG_LED_PREFIX)]){
        case 'H':
        case 'h':
        case '1':
            gpio_set_pin_level(RED_LED, true);
            break;
        case 'L':
        case 'l':
        case '0':
            gpio_set_pin_level(RED_LED, false);
            break;
        }
    }

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Callbacks
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool pccomm_cb_usb_state(usb_cdc_control_signal_t state){
    if (state.rs232.DTR) {
        // Register callbacks **AFTER** endpoints allocated
        cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, (FUNC_PTR)pccomm_cb_usb_read);
        cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, (FUNC_PTR)pccomm_cb_usb_write);

        // Initial read
        // Any data read will be available in the read callback
        // The next read should occur in the read callback
        cdcdf_acm_read((uint8_t *)pccomm_rx_buf, PCCOMM_RX_BUF_SZ);
    }

    // No error
    return false;
}

// Called when read completes. "count" is number of bytes that were read
bool pccomm_cb_usb_read(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count){
    // Handle previously read data
    for(uint32_t i = 0; i < count; ++i){
        if(pccomm_parse_escaped){
            // Take only valid escaped data
            // Invalid escaped bytes will be ignored
            if(pccomm_rx_buf[i] == PCCOMM_START_BYTE || pccomm_rx_buf[i] == PCCOMM_END_BYTE 
                    || pccomm_rx_buf[i] == PCCOMM_ESCAPE_BYTE){
                pccomm_recv_msg[pccomm_recv_msg_pos++] = pccomm_rx_buf[i];
            }
            pccomm_parse_escaped = false;
        }else{
            if(pccomm_rx_buf[i] == PCCOMM_START_BYTE){
                if(pccomm_parse_started){
                    // Second start byte before end byte
                    // Discard previously received data
                    pccomm_recv_msg_pos = 0;
                }
                pccomm_parse_started = true;
            }else if(pccomm_rx_buf[i] == PCCOMM_END_BYTE && pccomm_parse_started){
                pccomm_parse_started = false;
                pccomm_handle_received_msg();
                pccomm_recv_msg_pos = 0;
            }else if(pccomm_rx_buf[i] == PCCOMM_ESCAPE_BYTE && pccomm_parse_started){
                pccomm_parse_escaped = true;
            }else if(pccomm_parse_started){
                pccomm_recv_msg[pccomm_recv_msg_pos++] = pccomm_rx_buf[i];
            }
        }
    }

    // Start next read
    cdcdf_acm_read((uint8_t*)pccomm_rx_buf, PCCOMM_RX_BUF_SZ);

    // No error
    return false;
}

// Called when write complets. "count" is number of bytes that were just written
bool pccomm_cb_usb_write(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count){
    // Move data from write buffer into current write array
    uint32_t i;
    for(i = 0; i < PCCOMM_TX_BUF_SZ; ++i){
        if(CB_EMPTY(&pccomm_tx_queue))
            break;  // Nothing else to transmit
        cb_read(&pccomm_tx_queue, &pccomm_tx_buf[i]);
    }

    // Start next write (if anything to write)
    if(i != 0){
        cdcdf_acm_write((uint8_t*)pccomm_tx_buf, i);
    }

    // No error
    return false;
}
