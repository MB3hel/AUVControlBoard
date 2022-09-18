#!/usr/bin/env python3

import serial
import time
from crccheck.crc import Crc16Ccitt


START_BYTE = b'\xfd'
END_BYTE = b'\xfe'
ESCAPE_BYTE = b'\xff'



def write_msg(ser: serial.Serial, msg: bytes):
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
        while True:
            write_msg(ser, b'RLED1')
            time.sleep(1)
            write_msg(ser, b'RLED0')
            time.sleep(1)
    except KeyboardInterrupt:
        pass
