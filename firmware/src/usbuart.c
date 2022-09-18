
#include <usbuart.h>
#include <circular_buffer.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define USBUART_CURR_RX_BUF_SZ          CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ
#define USBUART_CURR_TX_BUF_SZ          CONF_USB_CDCD_ACM_DATA_BULKOUT_MAXPKSZ
#define USBUART_TX_MSG_BUF_SZ           128
#define USBUART_RX_MSG_BUF_SZ           128


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Forward declared callback functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool usbuart_state_callback(usb_cdc_control_signal_t state);
bool usbuart_read_callback(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count);
bool usbuart_write_callback(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count);


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool usbuart_initialized = false;

volatile uint8_t usbuart_curr_tx[USBUART_CURR_TX_BUF_SZ];       // Array to hold data currently being written
volatile uint8_t usbuart_curr_rx[USBUART_CURR_RX_BUF_SZ];       // Array to hold data currently being read
volatile uint8_t usbuart_tx_array[USBUART_TX_MSG_BUF_SZ];       // Backing array for TX buffer
volatile uint8_t usbuart_rx_array[USBUART_RX_MSG_BUF_SZ];       // Backing array for RX buffer
volatile circular_buffer usbuart_tx_buf;                        // Ring buffer of data to transmit
volatile circular_buffer usbuart_rx_buf;                        // Ring buffer holding data read


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool usbuart_init(void){
    if(!usbuart_initialized){
        if(cdcdf_acm_is_enabled()){
            cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usbuart_state_callback);
            usbuart_initialized = true;
        }
    }
    return usbuart_initialized;
}

unsigned int usbuart_write(uint8_t *data, unsigned int len){
    unsigned int i;
    
    // Add data into write buffer
    for(i = 0; i < len; ++i){
        if(CB_FULL(&usbuart_tx_buf))
            break;
        cb_write(&usbuart_tx_buf, data[i]);
    }

    // Number of bytes actually writeen (buffer may have become full)
    return i;
}

unsigned int usbuart_read(uint8_t *data, unsigned int len){
    unsigned int i = 0;

    // Remove data from read buffer
    for(i = 0; i < len; ++i){
        if(CB_EMPTY(&usbuart_rx_buf))
            break;
        cb_read(&usbuart_rx_buf, &data[i]);
    }

    // Number of bytes actually read (buffer may have become empty)
    return i;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Callbacks
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool usbuart_state_callback(usb_cdc_control_signal_t state){
    if (state.rs232.DTR) {
        // Register callbacks **AFTER** endpoints allocated
        cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, (FUNC_PTR)usbuart_read_callback);
        cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, (FUNC_PTR)usbuart_write_callback);

        // Initial write
        // Nothing is actually written, but the write callback is invoked when done
        // So next write should occur in the write callback
        cdcdf_acm_write((uint8_t *)usbuart_curr_tx, 0);

        // Initial read
        // Any data read will be available in the read callback
        // The next read should occur in the read callback
        cdcdf_acm_read((uint8_t *)usbuart_curr_rx, USBUART_CURR_RX_BUF_SZ);
    }
    return false;
}

// Called when read completes. "count" is number of bytes that were read
bool usbuart_read_callback(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count){
    // Move previously read data into read buffer
    for(uint32_t i = 0; i < count; ++i){
        cb_write(&usbuart_rx_buf, usbuart_rx_array[i]);
    }

    // Start next read
    cdcdf_acm_read((uint8_t *)usbuart_curr_rx, USBUART_CURR_RX_BUF_SZ);

    // No error
    return false;
}

// Called when write complets. "count" is number of bytes that were written
bool usbuart_write_callback(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count){    
    // Remove previously writen data from the tx buffer
    uint32_t pos;
    if(count < USBUART_CURR_TX_BUF_SZ){
        // Not all bytes were transmitted.
        // Move ones not yet transmitted to front of curr tx array
        uint32_t dest = 0;
        uint32_t src = count;
        while(src < USBUART_CURR_TX_BUF_SZ){
            usbuart_curr_tx[dest] = usbuart_curr_tx[src];
            src++;
            dest++;
        }

        // Set pos correctly
        pos = count;
    }else{
        // All data previously written. No need to keep anything from previous curr tx array
        pos = 0;
    }

    // Fill rest of curr tx array
    for(; pos < CONF_USB_CDCD_ACM_DATA_BULKOUT_MAXPKSZ; ++pos){
        if(CB_EMPTY(&usbuart_rx_buf))
            break;
        cb_read(&usbuart_tx_buf, &usbuart_curr_tx[pos]);
    }

    // Start next write (however many bytes in tx buffer up to limit for usb cdc)
    // Writes 0 bytes if nothing in tx buffer just so this funciton is called again later
    cdcdf_acm_write((uint8_t *)usbuart_curr_tx, pos);

    // No error
    return false;
}
