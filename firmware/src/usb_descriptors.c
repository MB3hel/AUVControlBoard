/**
 * USB descriptors used by TinyUSB
 * HIGH SPEED mode things removed since TinyUSB does not support HIGH speed on SAMD51
 * https://docs.tinyusb.org/en/latest/reference/supported.html
 * 
 * @file usb_descriptors.c
 * @author Marcus Behel
 */

#include "tusb.h"

// These need to not change after first use or driver may require manual uninstall
// See TinyUSB examples usb_descriptors.c for more details
// Using base IDs of Adafruit ItsyBitsy M4 Express (VID and PID)
#define _PID_MAP(itf, n)  ( (CFG_TUD_##itf) << (n) )
#define USB_PID           (0x002B | _PID_MAP(CDC, 0) | _PID_MAP(MSC, 1) | _PID_MAP(HID, 2) | \
                            _PID_MAP(MIDI, 3) | _PID_MAP(VENDOR, 4))
#define USB_VID   0x239A
#define USB_BCD   0x0200

//------------------------------------------------------------------------------
// Device Descriptor
//------------------------------------------------------------------------------

tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,

    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

uint8_t const * tud_descriptor_device_cb(void){
    // Invoked on GET DEVICE DESCRIPTOR request
    return (uint8_t const *) &desc_device;
}

//------------------------------------------------------------------------------
// Configuration Descriptor
//------------------------------------------------------------------------------

// If other interfaces used, add here
enum{
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

// Default options which end up being used for samd51 in TinyUSB examples
#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x82

// If other classes / interfaces enabled, add descriptors length here
#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

uint8_t const desc_fs_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64)

    // If other classes / interfaces used, add descriptors here
};

uint8_t const * tud_descriptor_configuration_cb(uint8_t index){
    // Invoked on GET DEVICE DESCRIPTOR request
    return desc_fs_configuration;
}

//------------------------------------------------------------------------------
// String Descriptors
//------------------------------------------------------------------------------

char const* string_desc_arr [] = {
    (const char[]) { 0x09, 0x04 },                  // 0: is supported language is English (0x0409)
    "Adafruit",                                     // 1: Manufacturer
    "SW8 Control Board (ItsyBitsy M4 Express)",     // 2: Product
    "",                                             // 3: Serials (empty, overriden in func)
    "ControlBoard CDC"                              // 4: CDC Interface
    // Additional classes need descriptors here too if enabled
};

static uint16_t _desc_str[64];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid){
    // Invoked on GET STRING DESCRIPTOR request
    
    uint8_t chr_count;

    if ( index == 0){
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    }else if(index == 3){
        // NOTE: SAMD51 specific
        // Get chip serial number as a string
        // Values of word are concatenated as hex strings
        // Concatenated word3, word2, word1, word0 order
        // Upper bits of each word first
        // Use this as serial descriptor string
        // Addresses from datasheet page 59
        const char *translation = 
                (const char[]){'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
        const volatile uint32_t *word0 = (uint32_t*)0x008061FC;
        const volatile uint32_t *word1 = (uint32_t*)0x00806010;
        const volatile uint32_t *word2 = (uint32_t*)0x00806014;
        const volatile uint32_t *word3 = (uint32_t*)0x00806018;
        uint32_t *data = (uint32_t[]){*word3, *word1, *word2, *word0};
        unsigned int idx = 1;
        for(unsigned int w = 0; w < 4; ++w){
            _desc_str[idx++] = translation[(data[w] & 0xF0000000) >> 28];
            _desc_str[idx++] = translation[(data[w] & 0x0F000000) >> 24];
            _desc_str[idx++] = translation[(data[w] & 0x00F00000) >> 20];
            _desc_str[idx++] = translation[(data[w] & 0x000F0000) >> 16];
            _desc_str[idx++] = translation[(data[w] & 0x0000F000) >> 12];
            _desc_str[idx++] = translation[(data[w] & 0x00000F00) >> 8];
            _desc_str[idx++] = translation[(data[w] & 0x000000F0) >> 4];
            _desc_str[idx++] = translation[(data[w] & 0x0000000F) >> 0];
        }
        chr_count = 32;
    }else{
        // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
        // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

        if ( !(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) ) return NULL;

        const char* str = string_desc_arr[index];

        // Cap at max char
        chr_count = (uint8_t) strlen(str);
        if ( chr_count > 63 ) chr_count = 63;

        // Convert ASCII string into UTF-16
        for(uint8_t i=0; i<chr_count; i++){
            _desc_str[1+i] = str[i];
        }
    }

    // first byte is length (including header), second byte is string type
    _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8 ) | (2*chr_count + 2));

    return _desc_str;
}

// -----------------------------------------------------------------------------
// Get chip serial number (specific to SAMD51)
// -----------------------------------------------------------------------------


