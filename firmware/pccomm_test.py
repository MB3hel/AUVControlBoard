#!/usr/bin/env python3

import serial
import threading
import time
from crccheck.crc import Crc16Ccitt


START_BYTE = b'\xfd'
END_BYTE = b'\xfe'
ESCAPE_BYTE = b'\xff'


def handle_read_message(msg: bytes):
    # Last two bytes of msg are crc

    # Ignore CRC for now
    # TODO: Verify CRC instead of ignoring
    msg = msg[:-2]

    print(msg)


def read_thread(ser: serial.Serial):
    global START_BYTE, END_BYTE, ESCAPE_BYTE
    parse_escaped = False
    parse_started = False
    data = bytearray()
    while True:
        c = ser.read(1)
        if parse_escaped:
            if c == START_BYTE or c == END_BYTE or c == ESCAPE_BYTE:
                data.extend(c)
            parse_escaped = False
        else:
            if c == START_BYTE:
                if parse_started:
                    data.clear()
                parse_started = True
            elif c == END_BYTE and parse_started:
                parse_started = False
                handle_read_message(bytes(data))
                data.clear()
            elif c == ESCAPE_BYTE and parse_started:
                parse_escaped = True
            elif parse_started:
                data.extend(c)


def write_msg(ser: serial.Serial, msg: bytes):
    global START_BYTE, END_BYTE, ESCAPE_BYTE
    ser.write(START_BYTE)
    for i in range(len(msg)):
        c = msg[i:i+1]
        if c == START_BYTE or c == END_BYTE or c == ESCAPE_BYTE:
            ser.write(ESCAPE_BYTE)
        ser.write(c)
    crc = Crc16Ccitt.calcbytes(msg, byteorder='big')
    ser.write(crc)
    ser.write(END_BYTE)


if __name__ ==  "__main__":
    try:
        ser = serial.Serial("/dev/ttyACM0")
        t = threading.Thread(target=read_thread, daemon=True, args=(ser,))
        t.start()
        while True:
            write_msg(ser, b'RLED1')
            time.sleep(1)
            write_msg(ser, b'RLED0')
            time.sleep(1)
    except KeyboardInterrupt:
        pass
