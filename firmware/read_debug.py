#!/usr/bin/env python3

import serial
from crccheck.crc import Crc16CcittFalse as Crc16


def handle_msg(msg: bytes):
    read_crc = int.from_bytes(msg[-2:], byteorder='big', signed=False)
    calc_crc = int.from_bytes(Crc16.calcbytes(msg[:-2], byteorder='big'), byteorder='big', signed=False)
    if read_crc == calc_crc:
        msg = msg[:-2]
        if msg.startswith(b'DEBUG'):
            print(msg[5:].decode())


def main():
    START_BYTE = b'\xfd'
    END_BYTE = b'\xfe'
    ESCAPE_BYTE = b'\xff'

    ser = serial.Serial("/dev/ttyACM0", 115200)
    parse_escaped = False
    parse_started = False
    data = bytearray()
    try:
        while True:
            c = b''
            try:
                c = ser.read(1)
            except (serial.SerialException, OSError) as e:
                print(flush=True)
                print("Communication lost!", flush=True)
                return 1
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
                    handle_msg(bytes(data))
                    data.clear()
                elif c == ESCAPE_BYTE and parse_started:
                    parse_escaped = True
                elif parse_started:
                    data.extend(c)
    except KeyboardInterrupt:
        return 0
    

if __name__ == "__main__":
    exit(main())
