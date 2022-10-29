/**
 * @file usb.c
 * @author Marcus Behel
 */

#include <usb.h>
#include <sam.h>
#include <clocks.h>
#include <tusb.h>
#include <timers.h>
#include <flags.h>
#include <util.h>
#include <dotstar.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MSG_QUEUE_SZ                6                               // Max number of messages in message queue

#define START_BYTE                  253                             // Communication  protocol start byte
#define END_BYTE                    254                             // Communication protocol end byte
#define ESCAPE_BYTE                 255                             // Communication protocol end byte

// Write c into the current message in the queue
#define WRITE_CURR_MSG(c)           msg_queue[msg_queue_widx][msg_queue_pos[msg_queue_widx]++] = (c)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static uint8_t msg_queue[MSG_QUEUE_SZ][USB_MAX_MSG_LEN+2];              // Holds complete received messages & crcs
static uint32_t msg_queue_pos[MSG_QUEUE_SZ];                        // Size of messages in each spot of queue
static uint32_t msg_queue_widx;                                     // Index in queue to place next message at
static uint32_t msg_queue_ridx;                                     // Index in queue to read next message from
static uint32_t msg_queue_count;                                    // Number of complete messages in queue


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void usb_init(void){
    MCLK->AHBMASK.bit.USB_ = 1;                                     // Enable AHB clock to USB
    MCLK->APBBMASK.bit.USB_ = 1;                                    // Enable APB clock to USB

    NVIC_SetPriority(USB_0_IRQn, 0UL);                              // Give USB interrupts highest priority
    NVIC_SetPriority(USB_1_IRQn, 0UL);                              // Give USB interrupts highest priority
    NVIC_SetPriority(USB_2_IRQn, 0UL);                              // Give USB interrupts highest priority
    NVIC_SetPriority(USB_3_IRQn, 0UL);                              // Give USB interrupts highest priority

    NVIC_EnableIRQ(USB_0_IRQn);                                     // Enable interrupt handlers for USB
    NVIC_EnableIRQ(USB_1_IRQn);                                     // Enable interrupt handlers for USB
    NVIC_EnableIRQ(USB_2_IRQn);                                     // Enable interrupt handlers for USB
    NVIC_EnableIRQ(USB_3_IRQn);                                     // Enable interrupt handlers for USB

    tud_init(BOARD_TUD_RHPORT);                                     // Initialize TinyUSB
}

void usb_process(void){
    static bool parse_started = false;
    static bool parse_escaped = false;

    tud_task();                                                     // TinyUSB task for device 

    // Parse a finite number of received bytes
    if (tud_cdc_connected() && tud_cdc_available()){
        uint8_t buf[64];
        uint32_t count = tud_cdc_read(buf, 64);
        // Only add data to message queue if it is not full
        // It should never fill up, but just in case.
        // If it fills up, probably needs to be larger
        if(msg_queue_count < MSG_QUEUE_SZ){
            for(uint32_t i = 0; i < count; ++i){
                // Handle one byte
                // This implements the parser
                if(parse_escaped){
                    // Previous was escape byte
                    // Handle valid escape sequences
                    // Ignore invalid escaped bytes
                    if(buf[i] == START_BYTE || buf[i] == END_BYTE || buf[i] == ESCAPE_BYTE)
                        WRITE_CURR_MSG(buf[i]);
                    parse_escaped = false;
                }else if(parse_started){
                    switch(buf[i]){
                    case START_BYTE:
                        // Got second start byte.
                        // Discard previous data (never got end byte)
                        msg_queue_pos[msg_queue_widx] = 0;
                        break;
                    case END_BYTE:
                        // Got a complete message.
                        if(msg_queue_pos[msg_queue_widx] >= 3){
                            // Must have at least one data byte + crc
                            volatile uint8_t *crc_data = &msg_queue[msg_queue_widx][msg_queue_pos[msg_queue_widx] - 2];
                            uint16_t read_crc = (crc_data[0] << 8) | crc_data[1];
                            uint16_t calc_crc = crc16_ccitt(msg_queue[msg_queue_widx], msg_queue_pos[msg_queue_widx] - 2);
                            if(read_crc == calc_crc){
                                // Valid message. Move widx to next spot in queue.
                                msg_queue_widx++;
                                if(msg_queue_widx >= MSG_QUEUE_SZ)
                                    msg_queue_widx = 0;
                                msg_queue_count++;

                                // Indicate to main tree that there is a message it should process
                                FLAG_SET(flags_main, FLAG_MAIN_USBMSG);
                            }else{
                                // Not a valid message. Clear queue spot
                                msg_queue_pos[msg_queue_widx] = 0;
                            }
                        }else{
                            // Not a valid message. Clear queue spot
                            msg_queue_pos[msg_queue_widx] = 0;
                        }
                        parse_started = false;
                        parse_escaped = false;
                        break;
                    case ESCAPE_BYTE:
                        parse_escaped = true;
                        break;
                    default:
                        WRITE_CURR_MSG(buf[i]);
                        break;
                    }
                }else if(buf[i] == START_BYTE){
                    parse_started = true;
                }
            }
        }
    }
}

uint32_t usb_getmsg(uint8_t *dest){
    uint32_t res;
    if(msg_queue_count == 0){
        // Nothing to get
        return 0;
    }

    // Copy next available message
    // Note: Exclude last two bytes because these are crc
    memcpy(dest, msg_queue[msg_queue_ridx], msg_queue_pos[msg_queue_ridx] - 2);

    // Clear this spot in queue (so next write will start at index 0)
    res = msg_queue_pos[msg_queue_ridx] - 2;
    msg_queue_pos[msg_queue_ridx] = 0;

    // Increment ridx (rolling over as needed)
    msg_queue_ridx++;
    if(msg_queue_ridx == MSG_QUEUE_SZ)
        msg_queue_ridx = 0;

    // Decrement counter
    msg_queue_count--;

    if(msg_queue_count != 0){
        // Still message(s) in the queue
        // Indicate to main tree that there is a message it should process
        FLAG_SET(flags_main, FLAG_MAIN_USBMSG);
    }

    return res;
}

inline static void __attribute__((always_inline)) usb_writeone(uint8_t c) {
    // If the write fails, the FIFO is full
    // Really this shouldn't happen. If it does, the FIFO size should probably
    // be increased in tusb_config.h
    // FIFO should at least be the size of one message. Probably multiple.
    // This will block until write succeeds
    while(tud_cdc_write_char(c) == 0){
        // Write failed.
        // Typically, TinyUSB will only send data once the FIFO reaches the bulk packet size
        // Calling this will cause a transfer to start now if one isn't already in progress
        // If a transfer is in progress (or there is no data to transfer), this function will return 0
        // Otherwise, it returns the number of bytes removed from the FIFO
        // Wait until a transfer can be started to continue writing
        while(tud_cdc_write_flush() == 0)
            delay_ms(1);        // Delay will feed watchdog inside this loop
        TIMERS_WDT_FEED();      // Don't reset the system while here
    }
}

void usb_writemsg(uint8_t *msg, uint32_t len){
    usb_writeone(START_BYTE);
    for(uint32_t i = 0; i < len; ++i){
        if(msg[i] == START_BYTE || msg[i] == END_BYTE || msg[i] == ESCAPE_BYTE)
            usb_writeone(ESCAPE_BYTE);
        usb_writeone(msg[i]);
    }

    // Calculate and add crc
    uint16_t crc = crc16_ccitt(msg, len);
    uint8_t high_byte = (crc >> 8) & 0xFF;
    uint8_t low_byte = crc & 0xFF;
    if(high_byte == START_BYTE || high_byte == END_BYTE || high_byte == ESCAPE_BYTE)
        usb_writeone(ESCAPE_BYTE);
    usb_writeone(high_byte);
    if(low_byte == START_BYTE || low_byte == END_BYTE || low_byte == ESCAPE_BYTE)
        usb_writeone(ESCAPE_BYTE);
    usb_writeone(low_byte);

    usb_writeone(END_BYTE);

    // Typically, TinyUSB will only send data once the FIFO reaches the bulk packet size
    // Calling this will cause a transfer to start now if one isn't already in progress
    // If a transfer is in progress (or there is no data to transfer), this function will return 0
    // Otherwise, it returns the number of bytes removed from the FIFO to start the transfer
    tud_cdc_write_flush();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// TinyUSB Callbacks
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts){
    // Run when line state changes
    // DTR = Data Terminal Ready
    // RTS = Ready to Send
    // DRT usually set when terminal connected

    // sam-ba upload protocol uses a 1200bps "touch" to trigger a reset
    // Handle this as expected
    // Not strictly necessary, but prevents having to press reset button to program
    // 1200bps "touch" means opening the port at 1200bps then closing it again (quickly)
    // Nothing is implemented here with timing. It just boots to bootloader when
    // a 1200bps connection is closed
    if (!dtr && itf == 0) {
        cdc_line_coding_t coding;
        tud_cdc_get_line_coding(&coding);
        if (coding.bit_rate == 1200){
            // Special things to reboot to bootloader instead of main program
            // Must match bootloader. Taken from Adafruit/ArduinoCore-samd Reset.cpp
            // THIS IS SPECIFIC TO ITSY BITSY M4!!!
            #define DOUBLE_TAP_MAGIC             0xf01669efUL
            #define BOOT_DOUBLE_TAP_ADDRESS     (HSRAM_ADDR + HSRAM_SIZE - 4)
            volatile unsigned long *a = (unsigned long *)BOOT_DOUBLE_TAP_ADDRESS;
            *a = DOUBLE_TAP_MAGIC;

            // Reset the system now
            TIMERS_WDT_RESET_NOW();
            dotstar_set(0, 0, 255);
        }
    }
}

void tud_cdc_rx_cb(uint8_t itf){
    // Run when CDC receives data from host
}

void tud_cdc_tx_complete_cb(uint8_t itf){
    // Run when transmit complete (no data in write fifo)
    // When this happens, attempt to start another transfer (prevents data sitting in TX FIFO)
    tud_cdc_write_flush();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// IRQ Handlers
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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