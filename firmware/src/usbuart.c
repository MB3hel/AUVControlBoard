
#include <usbuart.h>

////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////

#define USBUART_RX_BUF_SIZE         CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ
#define USBUART_TX_BUF_SIZE         CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ


////////////////////////////////////////////////////////////////////////////////
/// Forward declared callback functions
////////////////////////////////////////////////////////////////////////////////

bool usbuart_state_callback(usb_cdc_control_signal_t state);
bool usbuart_read_callback(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count);
bool usbuart_write_callback(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count);


////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////

bool usbuart_initialized = false;

uint8_t usbuart_rx_buf[USBUART_RX_BUF_SIZE];
uint32_t usbuart_rx_buf_len = 0;

uint8_t usbuart_tx_buf[USBUART_TX_BUF_SIZE];
uint32_t usbuart_tx_buf_len = 0;


////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////

bool usbuart_init(void){
    if(!usbuart_initialized){
        if(cdcdf_acm_is_enabled()){
            gpio_set_pin_level(RED_LED, true);
            cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usbuart_state_callback);
            usbuart_initialized = true;
        }
    }
    return usbuart_initialized;
}

void usbuart_write(uint8_t *data, size_t len){
    // TODO
}

size_t usbuart_read(uint8_t *data, size_t len){
    // TODO
    return 0;
}


////////////////////////////////////////////////////////////////////////////////
/// Callbacks
////////////////////////////////////////////////////////////////////////////////

bool usbuart_state_callback(usb_cdc_control_signal_t state){
    if (state.rs232.DTR) {
        // Register callbacks **AFTER** endpoints allocated
        cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, (FUNC_PTR)usbuart_read_callback);
        cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, (FUNC_PTR)usbuart_write_callback);

        // Initial write
        // Nothing is actually written, but the write callback is invoked when done
        // So next write should occur in the write callback
        cdcdf_acm_write((uint8_t *)usbuart_tx_buf, 0);

        // Initial read
        // Any data read will be available in the read callback
        // The next read should occur in the read callback
        cdcdf_acm_read((uint8_t *)usbuart_rx_buf, USBUART_RX_BUF_SIZE);
    }
    return false;
}

// Called when read completes. "count" is number of bytes that were read
bool usbuart_read_callback(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count){
    // TODO: Move previously read  data into read buffer
    // TODO: Start next read by calling cdcdf_acm_read()

    // No error
    return false;
}

// Called when write complets. "count" is number of bytes that were written
bool usbuart_write_callback(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count){
    // TODO: Remove previously writen data from the tx buffer
    // TODO: Start next write (however many bytes in tx buffer up to limit for usb cdc)
    //       Write 0 bytes if nothing in tx buffer just so this funciton is called again

    // No error
    return false;
}
