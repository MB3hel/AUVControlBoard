
import serial
import struct
import threading
from typing import List, Dict


START_BYTE = b'\xfd'
END_BYTE = b'\xfe'
ESCAPE_BYTE = b'\xff'


class ControlBoard:
    ## Open communication with a control board
    #  @param port Serial port to communicate with control board by
    def __init__(self, port: str, debug = False):
        self.__id_mutex = threading.Lock()
        self.__msg_id = 0
        self.__debug = debug
        self.__ser = serial.Serial(port, 115200)
        self.__ser.reset_input_buffer()
        self.__stop = False
        self.__ack_conds: Dict[int, threading.Condition] = {}
        self.__read_thread = threading.Thread(target=self.__read_task, daemon=True)
        self.__read_thread.start()

    def __del__(self):
        self.__stop = True
        self.__read_thread.join()
        self.__ser.close()

    ## Calculate the 16-bit CCITT-FALSE CRC of the given data
    #  @param msg The data to calculate CRC of
    #  @param initial Initial CRC value to use (use 65535 by default)
    def __crc16_ccitt_false(self, msg: bytes, initial = 0xFFFF):
        crc = initial
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

    ## Handle a message read from control board
    #  @param msg Message read from control board
    def __handle_read_message(self, msg: bytes):
        pass

    ## Thread to repeatedly read from the control board serial port
    def __read_task(self):
        # Holds message being received
        msg = bytearray()

        # Track parse state
        parse_escaped = False
        parse_started = True

        while not self.__stop:
            # Blocks until  a byte is available
            b = self.__ser.read()

            # TODO: Implement this

    ## Wait to receive ack from control board
    #  @param msg_id ID of message to wait for ack
    #  @param timeout Time in seconds to wait for ack
    def __wait_for_ack(msg_id: int, timeout: float):
        # TODO: Implement this with threading.Condition
        pass

    ## Write one byte via serial
    #  @param b Single byte to write
    def __write_one(self, b: bytes):
        self.__ser.write(b)
        if self.__debug:
            print("WB: {}".format(b))

    ## Send a message to control board (properly encoded)
    #  @param msg Raw message (payload bytes) to send
    def __write_msg(self, msg: bytes):
        global START_BYTE, END_BYTE, ESCAPE_BYTE    

        # Generate the ID for this message and increment the global ID counter
        msg_id = -1
        with self.__id_mutex:
            msg_id = self.__msg_id
            self.__msg_id += 1
            if self.__msg_id > 65535:
                self.__msg_id = 0

        if self.__debug:
            print("WRITE: {}".format(msg))

        # Write start byte
        self.__write_one(START_BYTE)

        # Write message ID (unsigned 16-bit int big endian). Escape as needed.
        id_dat = struct.pack(">H", msg_id)
        b = id_dat[0:1]
        if b == START_BYTE or b == END_BYTE or b == ESCAPE_BYTE:
            self.__write_one(ESCAPE_BYTE)
        self.__write_one(b)
        b = id_dat[1:2]
        if b == START_BYTE or b == END_BYTE or b == ESCAPE_BYTE:
            self.__write_one(ESCAPE_BYTE)
        self.__write_one(b)

        # Write each byte of msg (escaping it as necessary)
        for i in range(len(msg)):
            b = msg[i:i+1]
            if b == START_BYTE or b == END_BYTE or b == ESCAPE_BYTE:
                self.__write_one(ESCAPE_BYTE)
            self.__write_one(b)
        
        # Calculate CRC and write it. CRC INCLUDES MESSAGE ID BYTES.
        # Each byte of CRC must also be escaped
        crc = self.__crc16_ccitt_false(msg, self.__crc16_ccitt_false(id_dat))
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
    def set_raw(self, speeds: List[float], timeout=0.1) -> bool:
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
        # TODO: Wait for ACK
        None


