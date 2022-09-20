#!/usr/bin/env python3

import serial
import time
import threading
from crccheck.crc import Crc16CcittFalse as Crc16
import struct


START_BYTE = b'\xfd'
END_BYTE = b'\xfe'
ESCAPE_BYTE = b'\xff'


unhandled_msgs = []


def handle_read_message(msg: bytes):
    global unhandled_msgs
    # Last two bytes of msg are crc

    # Verify CRC
    read_crc = int.from_bytes(msg[-2:], byteorder='big', signed=False)
    calc_crc = int.from_bytes(Crc16.calcbytes(msg[:-2], byteorder='big'), byteorder='big', signed=False)

    if read_crc != calc_crc:
        print("WARNING: Invalid CRC!")
        return

    # Done with crc data
    msg = msg[:-2]

    unhandled_msgs.append(msg)


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
    crc = Crc16.calcbytes(msg, byteorder='big')
    ser.write(crc)
    ser.write(END_BYTE)


def wait_for_msg(msg: bytes):
    global unhandled_msgs
    while True:
        if msg in unhandled_msgs:
            return
        time.sleep(0.05)


def raw_speed_msg(s1, s2, s3, s4, s5, s6, s7, s8):
    msg = bytearray()
    msg.extend(b'RAW')
    msg.extend(struct.pack("<f", s1))
    msg.extend(struct.pack("<f", s2))
    msg.extend(struct.pack("<f", s3))
    msg.extend(struct.pack("<f", s4))
    msg.extend(struct.pack("<f", s5))
    msg.extend(struct.pack("<f", s6))
    msg.extend(struct.pack("<f", s7))
    msg.extend(struct.pack("<f", s8))
    return msg


if __name__ ==  "__main__":
    try:
        ser = serial.Serial("/dev/ttyACM0")
        t = threading.Thread(target=read_thread, daemon=True, args=(ser,))
        t.start()

        print("Set MODE = RAW")
        write_msg(ser, b'MODER')
        write_msg(ser, b'?MODE')
        wait_for_msg(b'MODER')
        print("MODE = RAW")
        
        print("SET SPEEDS")
        write_msg(ser, raw_speed_msg(-1, 1, -0.5, 0.5, -0.9, 0.9, -0.7, 0.7))
        print("SPEEDS SET")

    except KeyboardInterrupt:
        pass
