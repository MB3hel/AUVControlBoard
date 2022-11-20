/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Scott Shawcroft for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if CFG_TUD_ENABLED && \
    (CFG_TUSB_MCU == OPT_MCU_SAMD11 || CFG_TUSB_MCU == OPT_MCU_SAMD21 || \
     CFG_TUSB_MCU == OPT_MCU_SAMD51 || CFG_TUSB_MCU == OPT_MCU_SAME5X || \
     CFG_TUSB_MCU == OPT_MCU_SAML22 || CFG_TUSB_MCU == OPT_MCU_SAML21)

#include "samd51g19a.h"
#include "device/dcd.h"

/*------------------------------------------------------------------*/
/* MACRO TYPEDEF CONSTANT ENUM
 *------------------------------------------------------------------*/
static TU_ATTR_ALIGNED(4) usb_device_desc_bank_registers_t sram_registers[8][2];

// Setup packet is only 8 bytes in length. However under certain scenario,
// USB DMA controller may decide to overwrite/overflow the buffer  with
// 2 extra bytes of CRC. From datasheet's "Management of SETUP Transactions" section
//    If the number of received data bytes is the maximum data payload specified by
//    PCKSIZE.SIZE minus one, only the first CRC data is written to the data buffer.
//    If the number of received data is equal or less than the data payload specified
//    by PCKSIZE.SIZE minus two, both CRC data bytes are written to the data buffer.
// Therefore we will need to increase it to 10 bytes here.
static TU_ATTR_ALIGNED(4) uint8_t _setup_packet[8+2];

// ready for receiving SETUP packet
static inline void prepare_setup(void)
{
  // Only make sure the EP0 OUT buffer is ready
  sram_registers[0][0].USB_ADDR = (uint32_t) _setup_packet;
  sram_registers[0][0].USB_PCKSIZE = (sram_registers[0][0].USB_PCKSIZE & ~USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE_Msk) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(sizeof(tusb_control_request_t));
  sram_registers[0][0].USB_PCKSIZE = (sram_registers[0][0].USB_PCKSIZE & ~USB_DEVICE_PCKSIZE_BYTE_COUNT_Msk) | USB_DEVICE_PCKSIZE_BYTE_COUNT(0);
}

// Setup the control endpoint 0.
static void bus_reset(void)
{
  // Max size of packets is 64 bytes.
  usb_device_desc_bank_registers_t* bank_out = &sram_registers[0][TUSB_DIR_OUT];
  bank_out->USB_PCKSIZE = (bank_out->USB_PCKSIZE & ~USB_DEVICE_PCKSIZE_SIZE_Msk) | USB_DEVICE_PCKSIZE_SIZE(3);
  usb_device_desc_bank_registers_t* bank_in = &sram_registers[0][TUSB_DIR_IN];
  bank_in->USB_PCKSIZE = (bank_out->USB_PCKSIZE & ~USB_DEVICE_PCKSIZE_SIZE_Msk) | USB_DEVICE_PCKSIZE_SIZE(3);

  usb_device_endpoint_registers_t* ep = &USB_REGS->DEVICE.DEVICE_ENDPOINT[0];
  ep->USB_EPCFG = USB_DEVICE_EPCFG_EPTYPE0(0x1) | USB_DEVICE_EPCFG_EPTYPE1(0x1);
  ep->USB_EPINTENSET = USB_DEVICE_EPINTENSET_TRCPT0(1) | USB_DEVICE_EPINTENSET_TRCPT1(1) | USB_DEVICE_EPINTENSET_RXSTP(1);

  // Prepare for setup packet
  prepare_setup();
}

/*------------------------------------------------------------------*/
/* Controller API
 *------------------------------------------------------------------*/
void dcd_init (uint8_t rhport)
{
  (void) rhport;

  // Reset to get in a clean state.
  USB_REGS->DEVICE.USB_CTRLA = (USB_REGS->DEVICE.USB_CTRLA & ~USB_CTRLA_SWRST_Msk) | USB_CTRLA_SWRST(1);
  while ((USB_REGS->DEVICE.USB_SYNCBUSY & USB_SYNCBUSY_SWRST_Msk) == 0) {}
  while ((USB_REGS->DEVICE.USB_SYNCBUSY & USB_SYNCBUSY_SWRST_Msk) == 1) {}

  USB_REGS->DEVICE.USB_PADCAL = 
      ((*((uint32_t*) SW0_FUSES_REGS->FUSES_SW0_WORD_1) & USB_PADCAL_TRANSP_Msk) >> USB_PADCAL_TRANSP_Pos) | 
      ((*((uint32_t*) SW0_FUSES_REGS->FUSES_SW0_WORD_1) & USB_PADCAL_TRANSN_Msk) >> USB_PADCAL_TRANSN_Pos) | 
      ((*((uint32_t*) SW0_FUSES_REGS->FUSES_SW0_WORD_1) & USB_PADCAL_TRIM_Msk) >> USB_PADCAL_TRIM_Pos);

  USB_REGS->DEVICE.USB_QOSCTRL = (USB_REGS->DEVICE.USB_QOSCTRL & ~USB_QOSCTRL_CQOS_Msk) | USB_QOSCTRL_CQOS(3);
  USB_REGS->DEVICE.USB_QOSCTRL = (USB_REGS->DEVICE.USB_QOSCTRL & ~USB_QOSCTRL_DQOS_Msk) | USB_QOSCTRL_DQOS(3);

  // Configure registers
  USB_REGS->DEVICE.USB_DESCADD = (uint32_t) &sram_registers;
  USB_REGS->DEVICE.USB_CTRLB = USB_DEVICE_CTRLB_SPDCONF_FS;
  USB_REGS->DEVICE.USB_CTRLA = USB_CTRLA_MODE_DEVICE | USB_CTRLA_ENABLE(1) | USB_CTRLA_RUNSTDBY(1);
  while ((USB_REGS->DEVICE.USB_SYNCBUSY & USB_SYNCBUSY_ENABLE_Msk) == 1) {}

  USB_REGS->DEVICE.USB_INTFLAG |= USB_REGS->DEVICE.USB_INTFLAG; // clear pending
  USB_REGS->DEVICE.USB_INTENSET = /* USB_DEVICE_INTENSET_SOF(1) | */ USB_DEVICE_INTENSET_EORST(1);
}

void dcd_int_enable(uint8_t rhport)
{
  (void) rhport;
  NVIC_EnableIRQ(USB_OTHER_IRQn);
  NVIC_EnableIRQ(USB_SOF_HSOF_IRQn);
  NVIC_EnableIRQ(USB_TRCPT0_IRQn);
  NVIC_EnableIRQ(USB_TRCPT1_IRQn);
}

void dcd_int_disable(uint8_t rhport)
{
  (void) rhport;
  NVIC_DisableIRQ(USB_TRCPT1_IRQn);
  NVIC_DisableIRQ(USB_TRCPT0_IRQn);
  NVIC_DisableIRQ(USB_SOF_HSOF_IRQn);
  NVIC_DisableIRQ(USB_OTHER_IRQn);
}

void dcd_set_address (uint8_t rhport, uint8_t dev_addr)
{
  (void) dev_addr;

  // Response with zlp status
  dcd_edpt_xfer(rhport, 0x80, NULL, 0);

  // DCD can only set address after status for this request is complete
  // do it at dcd_edpt0_status_complete()

  // Enable SUSPEND interrupt since the bus signal D+/D- are stable now.
  USB_REGS->DEVICE.USB_INTFLAG = USB_DEVICE_INTENCLR_SUSPEND(1); // clear pending
  USB_REGS->DEVICE.USB_INTENSET = USB_DEVICE_INTENSET_SUSPEND(1);
}

void dcd_remote_wakeup(uint8_t rhport)
{
  (void) rhport;
  USB_REGS->DEVICE.USB_CTRLB |= USB_DEVICE_CTRLB_UPRSM_Msk;
}

// disconnect by disabling internal pull-up resistor on D+/D-
void dcd_disconnect(uint8_t rhport)
{
  (void) rhport;
  USB_REGS->DEVICE.USB_CTRLB |= USB_DEVICE_CTRLB_DETACH(1);
}

// connect by enabling internal pull-up resistor on D+/D-
void dcd_connect(uint8_t rhport)
{
  (void) rhport;
   USB_REGS->DEVICE.USB_CTRLB &= ~USB_DEVICE_CTRLB_DETACH(1);
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
  (void) rhport;
  (void) en;

  // TODO implement later
}

/*------------------------------------------------------------------*/
/* DCD Endpoint port
 *------------------------------------------------------------------*/

// Invoked when a control transfer's status stage is complete.
// May help DCD to prepare for next control transfer, this API is optional.
void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const * request)
{
  (void) rhport;

  if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE &&
      request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD &&
      request->bRequest == TUSB_REQ_SET_ADDRESS )
  {
    uint8_t const dev_addr = (uint8_t) request->wValue;
    USB_REGS->DEVICE.USB_DADD = USB_DEVICE_DADD_DADD(dev_addr) | USB_DEVICE_DADD_ADDEN(1);
  }

  // Just finished status stage, prepare for next setup packet
  // Note: we may already prepare setup when queueing the control status.
  // but it has no harm to do it again here
  prepare_setup();
}

bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const * desc_edpt)
{
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(desc_edpt->bEndpointAddress);
  uint8_t const dir   = tu_edpt_dir(desc_edpt->bEndpointAddress);

  usb_device_desc_bank_registers_t* bank = &sram_registers[epnum][dir];
  uint32_t size_value = 0;
  while (size_value < 7) {
    if (1 << (size_value + 3) == tu_edpt_packet_size(desc_edpt)) {
      break;
    }
    size_value++;
  }

  // unsupported endpoint size
  if ( size_value == 7 && tu_edpt_packet_size(desc_edpt) != 1023 ) return false;

  bank->USB_PCKSIZE = (bank->USB_PCKSIZE & ~USB_DEVICE_PCKSIZE_SIZE_Msk) | USB_DEVICE_PCKSIZE_SIZE(size_value);

  usb_device_endpoint_registers_t* ep = &USB_REGS->DEVICE.DEVICE_ENDPOINT[epnum];

  if ( dir == TUSB_DIR_OUT )
  {
    ep->USB_EPCFG = (ep->USB_EPCFG & ~USB_DEVICE_EPCFG_EPTYPE0_Msk) | USB_DEVICE_EPCFG_EPTYPE0(desc_edpt->bmAttributes.xfer + 1);
    ep->USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_STALLRQ0(1) | USB_DEVICE_EPSTATUSCLR_DTGLOUT(1); // clear stall & dtoggle
    ep->USB_EPINTENSET |= USB_DEVICE_EPINTENSET_TRCPT0_Msk;
  }else
  {
    ep->USB_EPCFG = (ep->USB_EPCFG & ~USB_DEVICE_EPCFG_EPTYPE1_Msk) | USB_DEVICE_EPCFG_EPTYPE1(desc_edpt->bmAttributes.xfer + 1);
    ep->USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_STALLRQ1(1) | USB_DEVICE_EPSTATUSCLR_DTGLIN(1); // clear stall & dtoggle
    ep->USB_EPINTENSET |= USB_DEVICE_EPINTENSET_TRCPT1_Msk;
  }

  return true;
}

void dcd_edpt_close (uint8_t rhport, uint8_t ep_addr) {
  (void) rhport;
  (void) ep_addr;

  // TODO: implement if necessary?
}

void dcd_edpt_close_all (uint8_t rhport)
{
  (void) rhport;
  // TODO implement dcd_edpt_close_all()
}

bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  usb_device_desc_bank_registers_t* bank = &sram_registers[epnum][dir];
  usb_device_endpoint_registers_t* ep = &USB_REGS->DEVICE.DEVICE_ENDPOINT[epnum];

  bank->USB_ADDR = (uint32_t) buffer;

  // A SETUP token can occur immediately after an ZLP Status.
  // So make sure we have a valid buffer for setup packet.
  //   Status = ZLP EP0 with direction opposite to one in the dir bit of current setup
  if ( (epnum == 0) && (buffer == NULL) && (total_bytes == 0) && (dir != tu_edpt_dir(_setup_packet[0])) ) {
    prepare_setup();
  }

  if ( dir == TUSB_DIR_OUT )
  {
    bank->USB_PCKSIZE = (bank->USB_PCKSIZE & ~USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE_Msk) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(total_bytes);
    bank->USB_PCKSIZE = (bank->USB_PCKSIZE & ~USB_DEVICE_PCKSIZE_BYTE_COUNT_Msk) | USB_DEVICE_PCKSIZE_BYTE_COUNT(0);
    ep->USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_BK0RDY(1);
    ep->USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRFAIL0(1);
  } else
  {
    bank->USB_PCKSIZE = (bank->USB_PCKSIZE & ~USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE_Msk) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(0);
    bank->USB_PCKSIZE = (bank->USB_PCKSIZE & ~USB_DEVICE_PCKSIZE_BYTE_COUNT_Msk) | USB_DEVICE_PCKSIZE_BYTE_COUNT(total_bytes);
    ep->USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_BK1RDY(1);
    ep->USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRFAIL1(1);
  }

  return true;
}

void dcd_edpt_stall (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);
  usb_device_endpoint_registers_t* ep = &USB_REGS->DEVICE.DEVICE_ENDPOINT[epnum];

  if (tu_edpt_dir(ep_addr) == TUSB_DIR_IN) {
    ep->USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_STALLRQ1(1);
  } else {
    ep->USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_STALLRQ0(1);
  }
}

void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);
  usb_device_endpoint_registers_t* ep = &USB_REGS->DEVICE.DEVICE_ENDPOINT[epnum];

  if (tu_edpt_dir(ep_addr) == TUSB_DIR_IN) {
    ep->USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_STALLRQ1(1) | USB_DEVICE_EPSTATUSCLR_DTGLIN(1);
  } else {
    ep->USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_STALLRQ0(1) | USB_DEVICE_EPSTATUSCLR_DTGLOUT(1);
  }
}

//--------------------------------------------------------------------+
// Interrupt Handler
//--------------------------------------------------------------------+
void maybe_transfer_complete(void) {
  uint32_t epints = USB_REGS->DEVICE.USB_EPINTSMRY;

  for (uint8_t epnum = 0; epnum < USB_EPT_NUM; epnum++) {
    if ((epints & (1 << epnum)) == 0) {
      continue;
    }

    usb_device_endpoint_registers_t* ep = &USB_REGS->DEVICE.DEVICE_ENDPOINT[epnum];
    uint32_t epintflag = ep->USB_EPINTFLAG;

    // Handle IN completions
    if ((epintflag & USB_DEVICE_EPINTFLAG_TRCPT1_Msk) != 0) {
      usb_device_desc_bank_registers_t* bank = &sram_registers[epnum][TUSB_DIR_IN];
      uint16_t const total_transfer_size = (bank->USB_PCKSIZE & USB_DEVICE_PCKSIZE_BYTE_COUNT_Msk) >> USB_DEVICE_PCKSIZE_BYTE_COUNT_Pos;

      dcd_event_xfer_complete(0, epnum | TUSB_DIR_IN_MASK, total_transfer_size, XFER_RESULT_SUCCESS, true);

      ep->USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRCPT1_Msk;
    }

    // Handle OUT completions
    if ((epintflag & USB_DEVICE_EPINTFLAG_TRCPT0_Msk) != 0) {
      usb_device_desc_bank_registers_t* bank = &sram_registers[epnum][TUSB_DIR_OUT];
      uint16_t const total_transfer_size = (bank->USB_PCKSIZE & USB_DEVICE_PCKSIZE_BYTE_COUNT_Msk) >> USB_DEVICE_PCKSIZE_BYTE_COUNT_Pos;

      dcd_event_xfer_complete(0, epnum, total_transfer_size, XFER_RESULT_SUCCESS, true);

      ep->USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRCPT0_Msk;
    }
  }
}


void dcd_int_handler (uint8_t rhport)
{
  (void) rhport;

  uint32_t int_status = USB_REGS->DEVICE.USB_INTFLAG & USB_REGS->DEVICE.USB_INTENSET;

  // Start of Frame
  if ( int_status & USB_DEVICE_INTFLAG_SOF_Msk )
  {
    USB_REGS->DEVICE.USB_INTFLAG = USB_DEVICE_INTFLAG_SOF_Msk;
    dcd_event_bus_signal(0, DCD_EVENT_SOF, true);
  }

  // SAMD doesn't distinguish between Suspend and Disconnect state.
  // Both condition will cause SUSPEND interrupt triggered.
  // To prevent being triggered when D+/D- are not stable, SUSPEND interrupt is only
  // enabled when we received SET_ADDRESS request and cleared on Bus Reset
  if ( int_status & USB_DEVICE_INTFLAG_SUSPEND_Msk )
  {
    USB_REGS->DEVICE.USB_INTFLAG = USB_DEVICE_INTFLAG_SUSPEND_Msk;

    // Enable wakeup interrupt
    USB_REGS->DEVICE.USB_INTFLAG = USB_DEVICE_INTFLAG_WAKEUP_Msk; // clear pending
    USB_REGS->DEVICE.USB_INTENSET = USB_DEVICE_INTFLAG_WAKEUP_Msk;

    dcd_event_bus_signal(0, DCD_EVENT_SUSPEND, true);
  }

  // Wakeup interrupt is only enabled when we got suspended.
  // Wakeup interrupt will disable itself
  if ( int_status & USB_DEVICE_INTFLAG_WAKEUP_Msk )
  {
    USB_REGS->DEVICE.USB_INTFLAG = USB_DEVICE_INTFLAG_WAKEUP_Msk;

    // disable wakeup interrupt itself
    USB_REGS->DEVICE.USB_INTENCLR = USB_DEVICE_INTFLAG_WAKEUP_Msk;
    dcd_event_bus_signal(0, DCD_EVENT_RESUME, true);
  }

  // Enable of Reset
  if ( int_status & USB_DEVICE_INTFLAG_EORST_Msk )
  {
    USB_REGS->DEVICE.USB_INTFLAG = USB_DEVICE_INTFLAG_EORST_Msk;

    // Disable both suspend and wakeup interrupt
    USB_REGS->DEVICE.USB_INTENCLR = USB_DEVICE_INTFLAG_WAKEUP_Msk | USB_DEVICE_INTFLAG_SUSPEND_Msk;

    bus_reset();
    dcd_event_bus_reset(0, TUSB_SPEED_FULL, true);
  }

  // Handle SETUP packet
  if (USB_REGS->DEVICE.DEVICE_ENDPOINT[0].USB_EPINTFLAG & USB_DEVICE_EPINTFLAG_RXSTP_Msk)
  {
    // This copies the data elsewhere so we can reuse the buffer.
    dcd_event_setup_received(0, _setup_packet, true);

    // Although Setup packet only set RXSTP bit,
    // TRCPT0 bit could already be set by previous ZLP OUT Status (not handled until now).
    // Since control status complete event is optional, we can just clear TRCPT0 and skip the status event
    USB_REGS->DEVICE.DEVICE_ENDPOINT[0].USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_RXSTP(1) | USB_DEVICE_EPINTFLAG_TRCPT0(1);
  }

  // Handle complete transfer
  maybe_transfer_complete();
}

#endif
