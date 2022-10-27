
#include <pccomm.h>

#include <sam.h>
#include <clocks.h>

#include "tusb.h"

void cdc_task(void);

void pccomm_init(void){
    GCLK->PCHCTRL[USB_GCLK_ID].bit.GEN = CLOCKS_GCLK_48M;
    GCLK->PCHCTRL[USB_GCLK_ID].bit.CHEN = 1;
    
    MCLK->AHBMASK.bit.USB_ = 1;
    MCLK->APBBMASK.bit.USB_ = 1;

	// Set up the USB DP/DN pins
    PORT->Group[0].PINCFG[PIN_PA24H_USB_DM].bit.PMUXEN = 1;
    PORT->Group[0].PMUX[PIN_PA24H_USB_DM / 2].reg &=
        ~(0xF << (4 * (PIN_PA24H_USB_DM & 0x01u)));
    PORT->Group[0].PMUX[PIN_PA24H_USB_DM / 2].reg |=
        MUX_PA24H_USB_DM << (4 * (PIN_PA24H_USB_DM & 0x01u));
    PORT->Group[0].PINCFG[PIN_PA25H_USB_DP].bit.PMUXEN = 1;
    PORT->Group[0].PMUX[PIN_PA25H_USB_DP / 2].reg &=
        ~(0xF << (4 * (PIN_PA25H_USB_DP & 0x01u)));
    PORT->Group[0].PMUX[PIN_PA25H_USB_DP / 2].reg |=
        MUX_PA25H_USB_DP << (4 * (PIN_PA25H_USB_DP & 0x01u));

	NVIC_SetPriority(USB_0_IRQn, 0UL);
	NVIC_SetPriority(USB_1_IRQn, 0UL);
	NVIC_SetPriority(USB_2_IRQn, 0UL);
	NVIC_SetPriority(USB_3_IRQn, 0UL);

    NVIC_EnableIRQ(USB_0_IRQn);
    NVIC_EnableIRQ(USB_1_IRQn);
    NVIC_EnableIRQ(USB_2_IRQn);
    NVIC_EnableIRQ(USB_3_IRQn);

	tud_init(BOARD_TUD_RHPORT);
}

void pccomm_process(void){
    tud_task();
    cdc_task();
}

void cdc_task(void)
{
  // connected() check for DTR bit
  // Most but not all terminal client set this when making connection
  // if ( tud_cdc_connected() )
  {
    // connected and there are data available
    if ( tud_cdc_available() )
    {
      // read datas
      char buf[64];
      uint32_t count = tud_cdc_read(buf, sizeof(buf));
      (void) count;

      // Echo back
      // Note: Skip echo by commenting out write() and write_flush()
      // for throughput test e.g
      //    $ dd if=/dev/zero of=/dev/ttyACM0 count=10000
      tud_cdc_write(buf, count);
      tud_cdc_write_flush();
    }
  }
}

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void) itf;
  (void) rts;

  // TODO set some indicator
  if ( dtr )
  {
    // Terminal connected
  }else
  {
    // Terminal disconnected
  }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
  (void) itf;
}

void USB_0_Handler (void){
    tud_int_handler(0);
}

void USB_1_Handler (void){
    tud_int_handler(0);
}

void USB_2_Handler (void){
    tud_int_handler(0);
}

void USB_3_Handler (void){
    tud_int_handler(0);
}