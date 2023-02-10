/*
 * Copyright 2022 Marcus Behel
 * 
 * This file is part of AUVControlBoard-Firmware.
 * 
 * AUVControlBoard-Firmware is free software: you can redistribute it and/or modify it under the terms of the GNU 
 * General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 * 
 * AUVControlBoard-Firmware is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even 
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for 
 * more details.
 * 
 * You should have received a copy of the GNU General Public License along with AUVControlBoard-Firmware. If not, see 
 * <https://www.gnu.org/licenses/>. 
 * 
 */

#include <pccomm.h>
#include <tusb.h>
#include <conversions.h>

// Comm protocol special bytes
#define START_BYTE          253
#define END_BYTE            254
#define ESCAPE_BYTE         255

uint8_t pccomm_read_buf[PCCOMM_MAX_MSG_LEN + 4];
unsigned int pccomm_read_len = 0;
uint16_t pccomm_read_crc = 0;

static uint16_t curr_msg_id = 0;


#define crc16_ccitt_false(data, len)      crc16_ccitt_false_partial((data), (len), 0xFFFF)  

/**
 * Calculate 16-bit CRC (CCITT-FALSE) of the given data
 * @param data Data to calculate crc of
 * @param len Length of data
 * @param initial Initial value to use in calc. Useful for fragmented data sets. 65535 for "default"
 * @return uint16_t Calculated crc
 */
uint16_t crc16_ccitt_false_partial(uint8_t *data, unsigned int len, uint16_t initial){
    uint16_t crc = initial;
    int pos = 0;
    while(pos < len){
        uint8_t b = data[pos];
        for(int i = 0; i < 8; ++i){
            uint8_t bit = ((b >> (7 - i) & 1) == 1);
            uint8_t c15 = ((crc >> 15 & 1) == 1);
            crc <<= 1;
            if(c15 ^ bit){
                crc ^= 0x1021;
            }
        }
        pos++;
    }
    return crc;
}


bool pccomm_read_and_parse(void){
    // Messages can be read & parsed over multiple calls
    // Thus, need to keep  track of current state and current message
    static bool parse_started = false;
    static bool parse_escaped = false;

    // Read available bytes one at a time and parse them
    uint8_t byte;
    while(tud_cdc_available()){        
        // Read the next available character
        byte = tud_cdc_read_char();

        // If message queue is full, discard bytes
        // Allow normal code to run for start byte because it will zero len
        if(byte != START_BYTE && pccomm_read_len == PCCOMM_MAX_MSG_LEN)
            continue;

        // Parse the meaning of this byte
        if(parse_escaped){
            // Currently escaped (previous byte was ESCAPE_BYTE)
            // Handle **valid** escape sequences (only special bytes can be escaped)
            // Ignore invalid escape sequences
            if(byte == START_BYTE || byte == END_BYTE || byte == ESCAPE_BYTE)
                pccomm_read_buf[pccomm_read_len++] = byte;
            
            // Handled the byte after an escape byte. No longer escaped
            parse_escaped = false;
        }else if(parse_started){
            // If a start byte was previously received, handle this byte
            switch(byte){
            case START_BYTE:
                // Handle start byte (special meaning when not escaped)
                // Discard old data when a start byte received
                pccomm_read_len = 0;
                break;
            case END_BYTE:
                // Handle end byte (special meaning when not escaped)
                // End byte means the buffer now holds the entire message

                parse_started = false;

                if(pccomm_read_len < 4){
                    // Too short to contain message id and crc bits. Invalid message.
                }else{
                    // Calculate CRC of read data. Exclude last two bytes.
                    // Last two bytes are the CRC (big endian) appended to the original data
                    // First two bytes are message ID. These are INCLUDED in CRC calc.
                    uint16_t calc_crc = crc16_ccitt_false(pccomm_read_buf, pccomm_read_len - 2);
                    pccomm_read_crc = conversions_data_to_int16(&pccomm_read_buf[pccomm_read_len - 2], false);

                    if(pccomm_read_crc == calc_crc){
                        // This is a complete, valid message
                        return true;
                    }else{
                        // Got a complete message, but it is invalid. Ignore it.
                        parse_started = false;
                    }
                }
                break;
            case ESCAPE_BYTE:
                // Handle escape byte (special meaning when not escaped)
                parse_escaped = true;
                break;
            default:
                // Handle normal bytes (these are just data)
                pccomm_read_buf[pccomm_read_len++] = byte;
                break;
            }
        }else if(byte == START_BYTE){
            // Received a start byte. Start parsing. Discard old data.
            parse_started = true;
            pccomm_read_len = 0;
        }
    }

    return false;
}

static inline void pccomm_write_one(uint8_t b){
    // Write one byte. If it fails, flush and try again
    if(!tud_cdc_write_char(b)){
        tud_cdc_write_flush();
        tud_cdc_write_char(b);
    }
}

void pccomm_write(uint8_t *msg, unsigned int len){
    taskENTER_CRITICAL();

    // Write start byte
    pccomm_write_one(START_BYTE);
    
    // Write message id (big endian). Escape it as needed.
    uint16_t msg_id = curr_msg_id;
    curr_msg_id++;
    uint8_t id_buf[2];
    conversions_int16_to_data(msg_id, id_buf, false);
    if(id_buf[0] == START_BYTE || id_buf[0] == END_BYTE || id_buf[0] == ESCAPE_BYTE)
        pccomm_write_one(ESCAPE_BYTE);
    pccomm_write_one(id_buf[0]);
    if(id_buf[1] == START_BYTE || id_buf[1] == END_BYTE || id_buf[1] == ESCAPE_BYTE)
        pccomm_write_one(ESCAPE_BYTE);
    pccomm_write_one(id_buf[1]);

    // Write each byte of msg (escaping it if necessary)
    for(unsigned int i = 0; i < len; ++i){
        if(msg[i] == START_BYTE || msg[i] == END_BYTE || msg[i] == ESCAPE_BYTE)
            pccomm_write_one(ESCAPE_BYTE);
        pccomm_write_one(msg[i]);
    }

    // Calculate CRC and write it. CRC INCLUDES MESSAGE ID BYTES!!!
    // Each byte of the CRC must also be escaped if it matches a special byte.
    uint16_t crc = crc16_ccitt_false(id_buf, 2);
    crc = crc16_ccitt_false_partial(msg, len, crc);
    uint8_t high_byte = (crc >> 8) & 0xFF;
    uint8_t low_byte = crc & 0xFF;
    if(high_byte == START_BYTE || high_byte == END_BYTE || high_byte == ESCAPE_BYTE)
        pccomm_write_one(ESCAPE_BYTE);
    pccomm_write_one(high_byte);
    if(low_byte == START_BYTE || low_byte == END_BYTE || low_byte == ESCAPE_BYTE)
        pccomm_write_one(ESCAPE_BYTE);
    pccomm_write_one(low_byte);

    // Write end byte
    pccomm_write_one(END_BYTE);

    // Write the message now
    tud_cdc_write_flush();

    taskEXIT_CRITICAL();
}
