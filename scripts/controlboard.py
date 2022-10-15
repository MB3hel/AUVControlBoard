
from typing import List
import serial
import time
import struct
import threading
from enum import Enum, auto
from crccheck.crc import Crc16CcittFalse as Crc16


class Quaternion:
    def __init__(self, w: float = 0.0, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.w: float = w
        self.w: float = x
        self.y: float = y
        self.z: float = z

class Vector3:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = x
        self.y = y
        self.z = z

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

        # Read any "old" data from the port
        # This could include old messages such as watchdog kills, sensor data, etc
        # From when control board was connected, but not in use
        # Thus messages just sat in buffer
        self.__ser.read_all()

        self.__state_lock = threading.Lock()
        self.__mode: ControlBoard.Mode = ControlBoard.Mode.UNKNOWN
        self.__inverted: List[int] = [2] * 8
        self.__read_thread = threading.Thread(target=self.__read_thread, daemon=True)
        self.__read_thread.start()
        self.__feed_thread = threading.Thread(target=self.__feed_thread, daemon=True)
        self.__feed_thread.start()

        self.__orientation_quat: Quaternion = Quaternion()
        self.__grav_vec: Vector3 = Vector3()

        self.set_mode(ControlBoard.Mode.RAW)
        self.set_inverted(False, False, False, False, False, False, False, False)

    def set_mode(self, mode: Mode) -> bool:
        with self.__state_lock:
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
        while True:
            with self.__state_lock:
                if self.__mode == mode:
                    return True
            if time.time() - start_time > 3.0:
                return False
            time.sleep(0.05)
    
    def get_mode(self) -> Mode:
        with self.__state_lock:
            return self.__mode
    
    def set_inverted(self, i1: bool, i2: bool, i3: bool, i4: bool, i5: bool, i6: bool, i7: bool, i8: bool) -> bool:
        newinv = [
            1 if i1 else 0,
            1 if i2 else 0,
            1 if i3 else 0,
            1 if i4 else 0,
            1 if i5 else 0,
            1 if i6 else 0,
            1 if i7 else 0,
            1 if i8 else 0
        ]

        with self.__state_lock:
            same = True
            for i in range(8):
                if newinv[i] != self.__inverted[i]:
                    same = False
                    break
            if same:
                # Inversions already correct
                return True
        
        # Construct and send message
        msg = bytearray()
        msg.extend(b'TINV')
        for i in range(8):
            msg.append(1 if newinv[i] else 0)
        self.__write_msg(msg)

        # Query inverted and wait for it to change (with a timeout)
        self.__write_msg(b'?TINV')
        start_time = time.time()
        while True:
            if time.time() - start_time > 3.0:
                return False
            with self.__state_lock:
                same = True
                for i in range(8):
                    if newinv[i] != self.__inverted[i]:
                        same = False
                        break
            if same:
                return True
            time.sleep(0.05)
    
    def get_inverted(self) -> List[bool]:
        with self.__state_lock:
            ret: List[bool] = []
            for i in range(8):
                ret.append(self.__inverted[i] == 1)
            return ret

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

    def set_local(self, x: float, y: float, z: float, pitch: float, roll: float, yaw: float):
        msg = bytearray()
        msg.extend(b'LOCAL')
        msg.extend(struct.pack("<f", x))
        msg.extend(struct.pack("<f", y))
        msg.extend(struct.pack("<f", z))
        msg.extend(struct.pack("<f", pitch))
        msg.extend(struct.pack("<f", roll))
        msg.extend(struct.pack("<f", yaw))
        self.__write_msg(msg)

    def get_gravity_vector(self) -> Vector3:
        return Vector3(self.__grav_vec.x, self.__grav_vec.y, self.__grav_vec.z)
    
    def get_orientation_quat(self) -> Quaternion:
        return Quaternion(self.__orientation_quat.w, self.__orientation_quat.x, 
                self.__orientation_quat.y, self.__orientation_quat.z)

    def __handle_read_message(self, msg: bytes):
        # Last two bytes of msg are crc
        # Verify CRC
        read_crc = int.from_bytes(msg[-2:], byteorder='big', signed=False)
        calc_crc = int.from_bytes(Crc16.calcbytes(msg[:-2], byteorder='big'), byteorder='big', signed=False)
        if read_crc != calc_crc:
            # Ignore messages with invalid CRC
            # Log that this occurred
            print("WARNING: Got a message with invalid CRC!")
            return

        # Done with crc data
        msg = msg[:-2]

        # Handle the message
        if msg.startswith(b'MODE'):
            # Mode status message
            # Response to mode query

            # Make sure enough data
            if len(msg) < 5:
                return

            # Parse data
            with self.__state_lock:
                if msg[4:5] == b'R':
                    self.__mode = ControlBoard.Mode.RAW
                elif msg[4:5] == b'L':
                    self.__mode = ControlBoard.Mode.LOCAL
        elif msg == b'WDGK':
            # Watchdog kill message
            # Received when motor watchdog times out and kills thrusters
            print("WARNING: Control Board Watchdog killed thrusters!")
        elif msg.startswith(b'TINV'):
            # Thruster inversion status message
            # Response to thruster inversion query

            # Make sure enough data
            if len(msg) < 12:
                return

            # Parse data
            with self.__state_lock:
                for i in range(8):
                    if msg[4+i] == 1:
                        self.__inverted[i] = True
                    else:
                        self.__inverted[i] = False

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
        crc = crc.replace(ControlBoard.START_BYTE, ControlBoard.ESCAPE_BYTE + ControlBoard.START_BYTE)
        crc = crc.replace(ControlBoard.ESCAPE_BYTE, ControlBoard.ESCAPE_BYTE + ControlBoard.ESCAPE_BYTE)
        crc = crc.replace(ControlBoard.END_BYTE, ControlBoard.ESCAPE_BYTE + ControlBoard.END_BYTE)
        self.__ser.write(crc)
        self.__ser.write(ControlBoard.END_BYTE)

    def __feed_thread(self):
        # Feed watchdog every 500ms
        # Watchdog on control board kills thrusters if not fed for 1500ms
        # to ensure motors do not continue running if the computer looses connection
        while True:
            self.__write_msg(b'WDGF')
            time.sleep(0.5)
