
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

        // Initialize TX
        cdcdf_acm_write((uint8_t *)usbuart_tx_buf, 0);

        // Initialize RX
        cdcdf_acm_read((uint8_t *)usbuart_rx_buf, USBUART_RX_BUF_SIZE);
    }
    return false;
}

bool usbuart_read_callback(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count){
    // TODO: Additional read calls here

    // No error
    return false;
}

bool usbuart_write_callback(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count){
    // TODO: Additional write calls here

    uint8_t *data = (uint8_t[]){'H', 'E', 'L', 'L', 'O', '\r', '\n'};
    cdcdf_acm_write(data, 7);

    // No error
    return false;
}
