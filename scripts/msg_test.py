
import struct
import time
from crccheck.crc import Crc16CcittFalse as Crc16
from enum import Enum, auto


class Mode(Enum):
    RAW = auto()
    LOCAL = auto()
    GLOBAL = auto()

START_BYTE = b'\xfd'
END_BYTE = b'\xfe'
ESCAPE_BYTE = b'\xff'

signed_bytes = True


def write_msg(msg: bytes):
    res = bytearray()
    res.extend(START_BYTE)
    for i in range(len(msg)):
        c = msg[i:i+1]
        if c == START_BYTE or c == END_BYTE or c == ESCAPE_BYTE:
            res.extend(ESCAPE_BYTE)
        res.extend(c)
    crc = Crc16.calcbytes(msg, byteorder='big')
    crc = crc.replace(START_BYTE, ESCAPE_BYTE + START_BYTE)
    crc = crc.replace(ESCAPE_BYTE, ESCAPE_BYTE + ESCAPE_BYTE)
    crc = crc.replace(END_BYTE, ESCAPE_BYTE + END_BYTE)
    res.extend(crc)
    res.extend(END_BYTE)
    
    print("[", end="")
    for i in range(len(res)):
        if signed_bytes:
            b = int.from_bytes(res[i:i+1], 'little', signed=True)
        else:
            b = res[i]
        print(b, end="")
        if i == len(res) - 1:
            print("]")
        else:
            print(",", end="")

def set_raw(s1: float, s2: float, s3: float, s4: float, s5: float, s6: float, s7: float, s8: float):
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
    write_msg(msg)

def set_local(x: float, y: float, z: float, pitch: float, roll: float, yaw: float):
    msg = bytearray()
    msg.extend(b'LOCAL')
    msg.extend(struct.pack("<f", x))
    msg.extend(struct.pack("<f", y))
    msg.extend(struct.pack("<f", z))
    msg.extend(struct.pack("<f", pitch))
    msg.extend(struct.pack("<f", roll))
    msg.extend(struct.pack("<f", yaw))
    write_msg(msg)

def set_global(self, x: float, y: float, z: float, pitch: float, roll: float, yaw: float):
    msg = bytearray()
    msg.extend(b'GLOBAL')
    msg.extend(struct.pack("<f", x))
    msg.extend(struct.pack("<f", y))
    msg.extend(struct.pack("<f", z))
    msg.extend(struct.pack("<f", pitch))
    msg.extend(struct.pack("<f", roll))
    msg.extend(struct.pack("<f", yaw))
    write_msg(msg)

def set_mode(mode: Mode) -> bool:
    # Construct and send mode set message
    msg = bytearray()
    msg.extend(b'MODE')
    if mode == Mode.RAW:
        msg.extend(b'R')
    elif mode == Mode.LOCAL:
        msg.extend(b'L')
    elif mode == Mode.GLOBAL:
        msg.extend(b'G')
    else:
        # Not a valid mode to set
        # Note that UNKNOWN mode cannot be set
        return False
    write_msg(bytes(msg))

def set_inverted(i1: bool, i2: bool, i3: bool, i4: bool, i5: bool, i6: bool, i7: bool, i8: bool) -> bool:
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
    
    # Construct and send message
    msg = bytearray()
    msg.extend(b'TINV')
    for i in range(8):
        msg.append(1 if newinv[i] else 0)
    write_msg(msg)

if __name__ == "__main__":
    set_mode(Mode.RAW)
    set_inverted(True, True, False, False, True, False, False, True)
    set_raw(0.5, 0, 0, 0, 0, 0, 0, 0)
    set_local(0, 0.5, 0, 0, 0, 0)