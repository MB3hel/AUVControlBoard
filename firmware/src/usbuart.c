
#include <usbuart.h>

////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////

#define USBUART_RX_BUF_SIZE         CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ / 4


////////////////////////////////////////////////////////////////////////////////
/// Forward declared callback functions
////////////////////////////////////////////////////////////////////////////////

bool usbuart_state_callback(usb_cdc_control_signal_t state);
bool usbuart_bulk_out_callback(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count);
bool usbuart_bulk_in_callback(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count);


////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////

bool usbuart_initialized = false;
uint8_t usbuart_rx_buffer[USBUART_RX_BUF_SIZE];


////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////

void usbuart_init(void){
    if(!usbuart_initialized){
        if(cdcdf_acm_is_enabled()){
            cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usbuart_state_callback);
        }
        usbuart_initialized = true;
    }
}

void usbuart_write(uint8_t *data, size_t len){

}

size_t usbuart_read(uint8_t *data, size_t len){

}


////////////////////////////////////////////////////////////////////////////////
/// Callbacks
////////////////////////////////////////////////////////////////////////////////

bool usbuart_state_callback(usb_cdc_control_signal_t state){
    if (state.rs232.DTR) {
        // Register callbacks **AFTER** endpoints allocated
		cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, (FUNC_PTR)usbuart_bulk_out_callback);
		cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, (FUNC_PTR)usbuart_bulk_in_callback);
		
        // Start data receive
		cdcdf_acm_read((uint8_t *)usbuart_rx_buffer, USBUART_RX_BUF_SIZE);
	}
	return false;
}

bool usbuart_bulk_out_callback(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count){
    // TODO
}

bool usbuart_bulk_in_callback(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count){
    // TODO
}
