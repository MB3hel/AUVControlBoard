# Communication Protocol

Communication with the control board relies on sending messages between the control board and PC. This section focuses on how messages are sent, not what messages are sent.


## Hardware Communication Layer

Messages are sent to the control board over the ItsyBitsy M4's builtin USB port. The control board acts as a USB ACM CDC device. In practice, this means that it shows up as a serial (UART) port on the computer it is connected to. However, baud rate settings are irrelevant (and changing baud rates has no effect). As such, messages are sent to / received from the control board using "UART" with an undefined baud rate. It is still necessary to set a baud rate when opening a UART port (as that information is provided to the device on the other side), but the rate is unused&ast;.

&ast;*The only exception is that you must NOT use a 1200 baud rate. Opening / closing the port at 1200bps is used to reset the control board for programming (boots to bootloader).*


## Message Format and Construction

The messages sent to / received from the control board have a specific format. Each message is a raw set of bytes (unsigned byte array). This set of bytes is the "payload data" of the message. The "payload data" is the data that is contained within a single message.

To be able to identify what data is part of a single message, it is necessary to add some additional information around the payload. The control board uses a special byte to indicate the start of a message (`START_BYTE`) and another one to identify the end of a message (`END_BYTE`). 

Since the payload could itself contain a start or end byte, there is also an escape byte (`ESCAPE_BYTE`) used to escape a `START_BYTE`, `END_BYTE`, or an `ESCAPE_BYTE` in the payload. 
- `START_BYTE` becomes `ESCAPE_BYTE`, `START_BYTE`
- `END_BYTE` becomes `ESCAPE_BYTE`, `END_BYTE`
- `ESCAPE_BYTE` becomes `ESCAPE_BYTE`, `ESCAPE_BYTE`

This is similar to escaping a quote in a string using a backslash.

For the control board:
- `START_BYTE` = 253 (unsigned 8-bit) = -3 (signed 8-bit)
- `END_BYTE` = 254 (unsigned 8-bit) = -2 (signed 8-bit)
- `ESCAPE_BYTE` = 255 (unsigned 8-bit) = -1 (signed 8-bit)

Additionally, each message contains a 16-bit CRC for the payload data (CCITT-FALSE algorithm). This CRC is calculated on the original (unescaped) payload data (no start byte, end byte, etc). The CRC is appended (big endian) to the message just after the payload (just before `END_BYTE`). When the other side receives the message it can calculate the CRC of the received payload and compare it to the received crc. If the crc values match, the message is not corrupt. Note that if the CRC bytes equal the start, end, or escape bytes, they too must be escaped.

<p align="center">
    <img height="175" src="./img/cb_msg_construction.png">
</p>