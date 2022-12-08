
import serial
import struct
from typing import List


START_BYTE = b'\xfd'
END_BYTE = b'\xfe'
ESCAPE_BYTE = b'\xff'


class ControlBoard:
    ## Open communication with a control board
    #  @param port Serial port to communicate with control board by
    def __init__(self, port: str, debug = False):
        self.__debug = debug
        self.__ser = serial.Serial(port, 115200)

    ## Calculate the 16-bit CCITT-FALSE CRC of the given data
    #  @param msg The data to calculate CRC of
    def __crc16_ccitt_false(self, msg: bytes):
        crc = 0xFFFF
        pos = 0
        while pos < len(msg):
            b = msg[pos]
            for i in range(8):
                bit = ((b >> (7 - i) & 1) == 1)
                c15 = ((crc >> 15 & 1) == 1)
                crc <<= 1
                crc &= 0xFFFF
                if c15 ^ bit:
                    crc ^= 0x1021
                    crc &= 0xFFFF
            pos += 1
        return crc & 0xFFFF

    def __write_one(self, b: bytes):
        self.__ser.write(b)
        if self.__debug:
            print("WB: {}".format(b))

    ## Send a message to control board (properly encoded)
    #  @param msg Raw message (payload bytes) to send
    def __write_msg(self, msg: bytes):
        global START_BYTE, END_BYTE, ESCAPE_BYTE

        if self.__debug:
            print("WRITE: {}".format(msg))

        # Write start byte
        self.__write_one(START_BYTE)

        # Write each byte of msg (escaping it as necessary)
        for i in range(len(msg)):
            b = msg[i:i+1]
            if b == START_BYTE or b == END_BYTE or b == ESCAPE_BYTE:
                self.__write_one(ESCAPE_BYTE)
            self.__write_one(b)
        
        # Calculate CRC and write it.
        # Each byte of CRC must also be escaped
        crc = self.__crc16_ccitt_false(msg)
        high_byte = (crc >> 8) & 0xFF
        low_byte = crc & 0xFF
        if high_byte == START_BYTE or high_byte == END_BYTE or high_byte == ESCAPE_BYTE:
            self.__write_one(ESCAPE_BYTE)
        self.__write_one(high_byte.to_bytes(1, 'little'))
        if low_byte == START_BYTE or low_byte == END_BYTE or low_byte == ESCAPE_BYTE:
            self.__write_one(ESCAPE_BYTE)
        self.__write_one(low_byte.to_bytes(1, 'little'))

        # Write end byte
        self.__write_one(END_BYTE)

    ## Set thruster speeds in RAW mode
    #  @param speeds List of 8 speeds to send to control board. Must range from -1 to 1
    def set_raw(self, speeds: List[float]):
        # Validate provided data
        if len(speeds) != 8:
            return
        for i in range(8):
            if speeds[i] < -1.0:
                speeds[i] = -1.0
            if speeds[i] > 1.0:
                speeds[i] = 1.0
        
        # Construct message to send
        data = bytearray()
        data.extend(b'RAW')
        data.extend(struct.pack("<f", speeds[0]))
        data.extend(struct.pack("<f", speeds[1]))
        data.extend(struct.pack("<f", speeds[2]))
        data.extend(struct.pack("<f", speeds[3]))
        data.extend(struct.pack("<f", speeds[4]))
        data.extend(struct.pack("<f", speeds[5]))
        data.extend(struct.pack("<f", speeds[6]))
        data.extend(struct.pack("<f", speeds[7]))
        self.__write_msg(bytes(data))


