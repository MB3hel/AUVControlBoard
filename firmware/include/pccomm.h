/**
 * @file pccomm.h
 * @brief Communication with PC using "UART" over builtin USB bus
 * 
 * Uses ASF4's USB ACM CDC driver to implement "UART" via USB CDC
 * 
 * Communication Protocol:
 * Messages can be sent across multiple "packets" meaning they may be sent / recieved in parts
 * Each message contains a "payload" which is the data that is actually being sent / received
 * The message will also contain other information used to identify messages
 * Each message starts with a START_BYTE and ends with an END_BYTE
 * Because the message payload can be arbitrary (meaning it could contain START_BYTE or END_BYTE)
 * there is also an ESCAPE_BYTE. If the payload contains a START_BYTE, END_BYTE, or ESCAPE_BYTE it
 * is escaped
 *      START_BYTE  --> ESCAPE_BYTE, START_BYTE
 *      END_BYTE    --> ESCAPE_BYTE, ESCAPE_BYTE
 *      ESCAPE_BYTE --> ESCAPE_BYTE, ESCAPE_BYTE
 * Additionally, each message contains a 16-bit CRC (CCITT) of the payload data that is used to ensure
 * messages are valid before preocessing them. These two crc bytes are appended to the end of the message
 * after the payload (high_byte, low_byte)
 * 
 * As such messages are in the following format
 *      START_BYTE, PAYLOAD, PAYLOAD, ..., CRC_HIGH, CRC_LOW, END_BYTE
 * 
 * When data is received, a CRC of the received payload should be calculated and compared to the received CRC.
 * If the two CRCs match, the message is valid. Otherwise the mesage is corrupt and should be ignored.
 *
 * Note that the CRC is always calculated on the "original" payload **not** on the escaped data
 * 
 * @author Marcus Behel (mgbehel@ncsu.edu)
 */

#pragma once


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void pccomm_init(void);
