/**
 * @file pccomm.c
 * @author Marcus Behel (mgbehel@ncsu.edu)
 */


#include <pccomm.h>
#include <atmel_start.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define PCCOMM_RX_BUF_SZ        64                  // Buffer for receiving data into
#define PCCOMM_RECV_MSG_SZ      128                 // Size of buffer to hold a received message

// Defined by communication protocol
#define PCCOMM_START_BYTE       253
#define PCCOMM_END_BYTE         254
#define PCCOMM_ESCAPE_BYTE      255


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t pccomm_rx_buf[PCCOMM_RX_BUF_SZ];
uint8_t pccomm_recv_msg[PCCOMM_RECV_MSG_SZ];
uint16_t pccomm_recv_msg_pos;
bool pccomm_parse_escaped;
bool pccomm_parse_started;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Callback forward declarations
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool pccomm_cb_usb_state(usb_cdc_control_signal_t state);
bool pccomm_cb_usb_read(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count);
bool pccomm_cb_usb_write(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count);


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void pccomm_init(void){
    pccomm_recv_msg_pos = 0;
    pccomm_parse_escaped = false;
    pccomm_parse_started = false;
    while(!cdcdf_acm_is_enabled()){
        delay_ms(100);
    }
    cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)pccomm_cb_usb_state);
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
                // TODO: Handle the complete message
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

bool pccomm_cb_usb_write(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count){
    // TODO: Write data if needed

    // No error
    return false;
}
