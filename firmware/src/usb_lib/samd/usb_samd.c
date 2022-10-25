#include <usb.h>
#include <samd/usb_samd.h>
#include <sam.h>

#define USB_EP_size_to_gc(x)  ((x <= 8   )?0:\
                               (x <= 16  )?1:\
                               (x <= 32  )?2:\
                               (x <= 64  )?3:\
                               (x <= 128 )?4:\
                               (x <= 256 )?5:\
                               (x <= 512 )?6:\
                                           7)

#define NVMC_CALIBRATION_AREA_ADDR		(0x00800080)
#define NVMC_CALIBRATION_AREA_DAT		(*((uint64_t*)(NVMC_CALIBRATION_AREA_ADDR)))

#undef ENABLE

void usb_init(){
	uint32_t pad_transn, pad_transp, pad_trim;

	// Enable APBB clock to USB
	MCLK->APBBMASK.bit.USB_ = 1;

	// Select GCLK for USB peripheral (must be 48MHz) and enable it
	// Define USB_GCLK_GEN in build options
	GCLK->PCHCTRL[USB_GCLK_ID].bit.GEN = USB_GCLK_GEN;
	GCLK->PCHCTRL[USB_GCLK_ID].bit.CHEN = 1; 

	// Reset USB device peripheral
	USB->DEVICE.CTRLA.reg = USB_CTRLA_SWRST;
	while (USB->DEVICE.SYNCBUSY.bit.SWRST);

	USB->DEVICE.CTRLA.reg = USB_CTRLA_ENABLE | USB_CTRLA_MODE_DEVICE;
	while (USB->DEVICE.SYNCBUSY.bit.ENABLE);

	// Load calibration data from NVM Software Calibration Area
	// See page 57-58 in datasheet and page 1033
	// If all bits are a 1, invalid cal data and use defaults
	pad_transn = (NVMC_CALIBRATION_AREA_DAT & ((uint64_t)0b11111 << 32)) >> 32;
	if (pad_transn == 0b11111) {
		pad_transn = 0x09;
	}
	pad_transp = (NVMC_CALIBRATION_AREA_DAT & ((uint64_t)0b11111 << 37)) >> 37;
	if (pad_transp == 0b11111) {
		pad_transp = 0x19;
	}
	pad_trim = (NVMC_CALIBRATION_AREA_DAT & ((uint64_t)0b111 << 42)) >> 42;
	if (pad_trim == 0b111) {
		pad_trim = 0x06;
	}
	USB->DEVICE.PADCAL.bit.TRANSN = pad_transn;
	USB->DEVICE.PADCAL.bit.TRANSP = pad_transp;
	USB->DEVICE.PADCAL.bit.TRIM = pad_trim;

	// Set base address of base address of the main USB descriptor in RAM
	// Lowest two bits must be zero (pg 1032 of datasheet)
	memset(usb_endpoints, 0, usb_num_endpoints*sizeof(UsbDeviceDescriptor));
	USB->DEVICE.DESCADD.reg = (uint32_t)(&usb_endpoints[0]);

	// Enable initial set of interrupts
	USB->DEVICE.INTENSET.reg = USB_DEVICE_INTENCLR_EORST;

	usb_reset();
}

#define USB_EPTYPE_DISABLED 0
#define USB_EPTYPE_CONTROL 1
#define USB_EPTYPE_ISOCHRONOUS 2
#define USB_EPTYPE_BULK 3
#define USB_EPTYPE_INTERRUPT 4
#define USB_EPTYPE_DUAL_BANK 5

void usb_reset(){
	usb_endpoints[0].DeviceDescBank[0].ADDR.reg = (uint32_t) &ep0_buf_out;
	usb_endpoints[0].DeviceDescBank[0].PCKSIZE.bit.SIZE = USB_EP_size_to_gc(USB_EP0_SIZE);
	usb_endpoints[0].DeviceDescBank[1].ADDR.reg = (uint32_t) &ep0_buf_in;
	usb_endpoints[0].DeviceDescBank[1].PCKSIZE.bit.SIZE=USB_EP_size_to_gc(USB_EP0_SIZE);
	usb_endpoints[0].DeviceDescBank[1].PCKSIZE.bit.AUTO_ZLP=1;
	USB->DEVICE.DeviceEndpoint[0].EPINTENSET.reg = USB_DEVICE_EPINTENSET_RXSTP;
	USB->DEVICE.DeviceEndpoint[0].EPCFG.reg  = USB_DEVICE_EPCFG_EPTYPE0(USB_EPTYPE_CONTROL)
	                                         | USB_DEVICE_EPCFG_EPTYPE1(USB_EPTYPE_CONTROL);
}

void usb_set_address(uint8_t addr) {
	USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | addr;
}

inline UsbDeviceDescBank* ep_ram(uint8_t epaddr) {
	return &usb_endpoints[epaddr&0x3F].DeviceDescBank[!!(epaddr&0x80)];
}

inline void usb_enable_ep(uint8_t ep, uint8_t type, usb_size bufsize) {
	if (ep & 0x80) {
		usb_endpoints[ep & 0x3f].DeviceDescBank[1].PCKSIZE.bit.SIZE = USB_EP_size_to_gc(bufsize);
		USB->DEVICE.DeviceEndpoint[ep & 0x3f].EPCFG.bit.EPTYPE1 = type + 1;
		USB->DEVICE.DeviceEndpoint[ep & 0x3f].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUS_BK1RDY
		                                                      | USB_DEVICE_EPSTATUS_STALLRQ(0x2)
		                                                      | USB_DEVICE_EPSTATUS_DTGLIN;
	} else {
		usb_endpoints[ep & 0x3f].DeviceDescBank[0].PCKSIZE.bit.SIZE = USB_EP_size_to_gc(bufsize);
		USB->DEVICE.DeviceEndpoint[ep & 0x3f].EPCFG.bit.EPTYPE0 = type + 1;
		USB->DEVICE.DeviceEndpoint[ep & 0x3f].EPSTATUSSET.reg = USB_DEVICE_EPSTATUS_BK0RDY;
		USB->DEVICE.DeviceEndpoint[ep & 0x3f].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUS_STALLRQ(0x1)
		                                                      | USB_DEVICE_EPSTATUS_DTGLOUT;
	}
}

inline void usb_disable_ep(uint8_t ep) {
	if (ep & 0x80) {
		USB->DEVICE.DeviceEndpoint[ep & 0x3f].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUS_BK1RDY;
		USB->DEVICE.DeviceEndpoint[ep & 0x3f].EPCFG.bit.EPTYPE1 = USB_EPTYPE_DISABLED;
	} else {
		USB->DEVICE.DeviceEndpoint[ep & 0x3f].EPSTATUSSET.reg = USB_DEVICE_EPSTATUS_BK0RDY;
		USB->DEVICE.DeviceEndpoint[ep & 0x3f].EPCFG.bit.EPTYPE0 = USB_EPTYPE_DISABLED;
	}
}

inline void usb_reset_ep(uint8_t ep){
	if (ep & 0x80) {
		USB->DEVICE.DeviceEndpoint[ep & 0x3f].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUS_BK1RDY;
	} else {
		USB->DEVICE.DeviceEndpoint[ep & 0x3f].EPSTATUSSET.reg = USB_DEVICE_EPSTATUS_BK0RDY;
	}
}

inline usb_bank usb_ep_start_out(uint8_t ep, uint8_t* data, usb_size len) {
	usb_endpoints[ep].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = len;
	usb_endpoints[ep].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;
	usb_endpoints[ep].DeviceDescBank[0].ADDR.reg = (uint32_t) data;
	USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT0 | USB_DEVICE_EPINTFLAG_TRFAIL0;
	USB->DEVICE.DeviceEndpoint[ep].EPINTENSET.reg = USB_DEVICE_EPINTENSET_TRCPT0;
	USB->DEVICE.DeviceEndpoint[ep].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUS_BK0RDY;
	return 0;
}

inline usb_bank usb_ep_start_in(uint8_t ep, const uint8_t* data, usb_size size, bool zlp) {
	ep &= 0x3f;
	usb_endpoints[ep].DeviceDescBank[1].PCKSIZE.bit.AUTO_ZLP = zlp;
	usb_endpoints[ep].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
	usb_endpoints[ep].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = size;
	usb_endpoints[ep].DeviceDescBank[1].ADDR.reg = (uint32_t) data;
	USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1 | USB_DEVICE_EPINTFLAG_TRFAIL1;
	USB->DEVICE.DeviceEndpoint[ep].EPINTENSET.reg = USB_DEVICE_EPINTENSET_TRCPT1;
	USB->DEVICE.DeviceEndpoint[ep].EPSTATUSSET.reg = USB_DEVICE_EPSTATUS_BK1RDY;
	return 0;
}

inline bool usb_ep_empty(uint8_t ep) {
	if (ep & 0x80) {
		return !(USB->DEVICE.DeviceEndpoint[ep & 0x3F].EPSTATUS.bit.BK1RDY || usb_ep_pending(ep));
	} else {
		return !(USB->DEVICE.DeviceEndpoint[ep & 0x3F].EPSTATUS.bit.BK0RDY || usb_ep_pending(ep));
	}
}

inline bool usb_ep_ready(uint8_t ep) {
	return usb_ep_empty(ep);
}

inline bool usb_ep_pending(uint8_t ep) {
	if (ep & 0x80) {
		return USB->DEVICE.DeviceEndpoint[ep & 0x3F].EPINTFLAG.bit.TRCPT1;
	} else {
		return USB->DEVICE.DeviceEndpoint[ep & 0x3F].EPINTFLAG.bit.TRCPT0;
	}
}

inline void usb_ep_handled(uint8_t ep) {
	if (ep & 0x80) {
		USB->DEVICE.DeviceEndpoint[ep & 0x3F].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1;
	} else {
		USB->DEVICE.DeviceEndpoint[ep & 0x3F].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT0;
	}
}

inline usb_size usb_ep_out_length(uint8_t ep){
	return usb_endpoints[ep].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT;
}

inline void usb_detach(void) {
	USB->DEVICE.CTRLB.bit.DETACH = 1;
	NVIC_DisableIRQ(USB_0_IRQn);
	NVIC_DisableIRQ(USB_1_IRQn);
	NVIC_DisableIRQ(USB_2_IRQn);
	NVIC_DisableIRQ(USB_3_IRQn);
}

inline void usb_attach(void) {
	NVIC_EnableIRQ(USB_0_IRQn);
	NVIC_EnableIRQ(USB_1_IRQn);
	NVIC_EnableIRQ(USB_2_IRQn);
	NVIC_EnableIRQ(USB_3_IRQn);
	USB->DEVICE.CTRLB.bit.DETACH = 0;
}

/// Enable the OUT stage on the default control pipe.
inline void usb_ep0_out(void) {
	usb_ep_start_out(0x00, ep0_buf_out, USB_EP0_SIZE);
}

inline void usb_ep0_in(uint8_t size){
	usb_ep_start_in(0x80, ep0_buf_in, size, true);
}

inline void usb_ep0_stall(void) {
	USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.reg = USB_DEVICE_EPSTATUS_STALLRQ(0x3);
}

void usb_set_speed(USB_Speed speed) {
	if (USB_SPEED_FULL == speed) {
		USB->DEVICE.CTRLB.bit.SPDCONF = USB_DEVICE_CTRLB_SPDCONF_FS_Val;
	} else if(USB_SPEED_LOW == speed) {
		USB->DEVICE.CTRLB.bit.SPDCONF = USB_DEVICE_CTRLB_SPDCONF_LS_Val;
	}
}

USB_Speed usb_get_speed() {
	if (USB->DEVICE.STATUS.bit.SPEED == 0) {
		return USB_SPEED_LOW;
	} else {
		return USB_SPEED_FULL;
	}
}

void USB_Handler_Common() {
	uint32_t summary = USB->DEVICE.EPINTSMRY.reg;
	uint32_t status = USB->DEVICE.INTFLAG.reg;

	if (status & USB_DEVICE_INTFLAG_EORST) {
		USB->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_EORST;
		usb_reset();
		usb_cb_reset();
		return;
	}

	if (summary & (1<<0)) {
		uint32_t flags = USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg;
		USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1 | USB_DEVICE_EPINTFLAG_TRCPT0 | USB_DEVICE_EPINTFLAG_RXSTP;
		if (flags & USB_DEVICE_EPINTFLAG_RXSTP) {
			memcpy(&usb_setup, ep0_buf_out, sizeof(usb_setup));
			usb_handle_setup();
		}
		if (flags & USB_DEVICE_EPINTFLAG_TRCPT0) {
			usb_handle_control_out_complete();
		}
		if (flags & USB_DEVICE_EPINTFLAG_TRCPT1) {
			usb_handle_control_in_complete();
		}
	}

	for (int i=1; i<usb_num_endpoints; i++) {
		if (summary & 1<<i) {
			uint32_t flags = USB->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg;
			USB->DEVICE.DeviceEndpoint[i].EPINTENCLR.reg = flags;
		}
	}

	usb_cb_completion();
}

void USB_0_Handler(void){
	USB_Handler_Common();
}

void USB_1_Handler(void){
	USB_Handler_Common();
}

void USB_2_Handler(void){
	USB_Handler_Common();
}

void USB_3_Handler(void){
	USB_Handler_Common();
}

void* samd_serial_number_string_descriptor() {
	char buf[27];

	const unsigned char* id = (unsigned char*) 0x008061FC;
	for (int i=0; i<26; i++) {
		unsigned idx = (i*5)/8;
		unsigned pos = (i*5)%8;
		unsigned val = ((id[idx] >> pos) | (id[idx+1] << (8-pos))) & ((1<<5)-1);
		buf[i] = "0123456789ABCDFGHJKLMNPQRSTVWXYZ"[val];
	}
	buf[26] = 0;
	return usb_string_to_descriptor(buf);
}
