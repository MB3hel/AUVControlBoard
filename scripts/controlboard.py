
import serial
import time
import struct
import threading
from enum import Enum, auto
from crccheck.crc import Crc16CcittFalse as Crc16


class ControlBoard:
    # Communication protocol special values
    START_BYTE = b'\xfd'
    END_BYTE = b'\xfe'
    ESCAPE_BYTE = b'\xff'

    # Modes of operation
    class Mode(Enum):
        UNKNOWN = auto()
        RAW = auto()
        LOCAL = auto()

    def __init__(self, port: str):
        self.__ser: serial.Serial = serial.Serial(port)
        self.__mode: ControlBoard.Mode = ControlBoard.Mode.UNKNOWN
        self.__read_thread = threading.Thread(target=self.__read_thread, daemon=True)
        self.__read_thread.start()

        self.set_mode(ControlBoard.Mode.RAW)

    def set_mode(self, mode: Mode) -> bool:
        if mode == self.__mode:
            # Already in the right mode
            return True

        # Construct and send mode set message
        msg = bytearray()
        msg.extend(b'MODE')
        if mode == ControlBoard.Mode.RAW:
            msg.extend(b'R')
        elif mode == ControlBoard.Mode.LOCAL:
            msg.extend(b'L')
        else:
            # Not a valid mode to set
            # Note that UNKNOWN mode cannot be set
            return False
        self.__write_msg(bytes(msg))

        # Query mode and wait for it to change (with a timeout)
        self.__write_msg(b'?MODE')
        start_time = time.time()
        while self.__mode != mode:
            if time.time() - start_time > 3.0:
                return False
            time.sleep(0.05)
        return True
    
    def get_mode(self) -> Mode:
        return self.__mode
    
    def set_raw(self, s1: float, s2: float, s3: float, s4: float, s5: float, s6: float, s7: float, s8: float):
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
        self.__write_msg(msg)

    def __handle_read_message(self, msg: bytes):
        # Last two bytes of msg are crc
        # Verify CRC
        read_crc = int.from_bytes(msg[-2:], byteorder='big', signed=False)
        calc_crc = int.from_bytes(Crc16.calcbytes(msg[:-2], byteorder='big'), byteorder='big', signed=False)
        if read_crc != calc_crc:
            # Ignore messages with invalid CRC
            # Log that this occured
            print("WARNING: Got a message with invalid CRC!")
            return

        # Done with crc data
        msg = msg[:-2]

        # Handle the message
        if msg.startswith(b'MODE'):
            # Mode status message
            # Response to mode query
            if msg[4:5] == b'R':
                self.__mode = ControlBoard.Mode.RAW
            elif msg[4:5] == b'L':
                self.__mode = ControlBoard.Mode.LOCAL

    def __read_thread(self):
        parse_escaped = False
        parse_started = False
        data = bytearray()
        while True:
            c = self.__ser.read(1)
            if parse_escaped:
                if c == ControlBoard.START_BYTE or c == ControlBoard.END_BYTE or c == ControlBoard.ESCAPE_BYTE:
                    data.extend(c)
                parse_escaped = False
            else:
                if c == ControlBoard.START_BYTE:
                    if parse_started:
                        data.clear()
                    parse_started = True
                elif c == ControlBoard.END_BYTE and parse_started:
                    parse_started = False
                    self.__handle_read_message(bytes(data))
                    data.clear()
                elif c == ControlBoard.ESCAPE_BYTE and parse_started:
                    parse_escaped = True
                elif parse_started:
                    data.extend(c)

    def __write_msg(self, msg: bytes):
        self.__ser.write(ControlBoard.START_BYTE)
        for i in range(len(msg)):
            c = msg[i:i+1]
            if c == ControlBoard.START_BYTE or c == ControlBoard.END_BYTE or c == ControlBoard.ESCAPE_BYTE:
                self.__ser.write(ControlBoard.ESCAPE_BYTE)
            self.__ser.write(c)
        crc = Crc16.calcbytes(msg, byteorder='big')
        self.__ser.write(crc)
        self.__ser.write(ControlBoard.END_BYTE)

