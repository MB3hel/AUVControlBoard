
from serial import Serial
from crccheck.crc import Crc16CcittFalse as Crc16


START_BYTE = b'\xfd'
END_BYTE = b'\xfe'
ESCAPE_BYTE = b'\xff'


def write_msg(ser: Serial, msg: bytes):
    global START_BYTE, END_BYTE, ESCAPE_BYTE
    ser.write(START_BYTE)
    for i in range(len(msg)):
        c = msg[i:i+1]
        if c == START_BYTE or c == END_BYTE or c == ESCAPE_BYTE:
            ser.write(ESCAPE_BYTE)
        ser.write(c)
    crc = Crc16.calcbytes(msg, byteorder='big')
    crc = crc.replace(START_BYTE, ESCAPE_BYTE + START_BYTE)
    crc = crc.replace(ESCAPE_BYTE, ESCAPE_BYTE + ESCAPE_BYTE)
    crc = crc.replace(END_BYTE, ESCAPE_BYTE + END_BYTE)
    ser.write(crc)
    ser.write(END_BYTE)
    


if __name__ == "__main__":
    ser = Serial("/dev/ttyACM0", 115200)
    while True:
        msg = input("Message: ")
        write_msg(ser, msg.encode())
