
import serial
import struct
import time
from enum import IntEnum
import threading
from typing import List, Dict


START_BYTE = b'\xfd'
END_BYTE = b'\xfe'
ESCAPE_BYTE = b'\xff'


class ControlBoard:

    class AckError(IntEnum):
        NONE = 0
        UNKNOWN_MSG = 1
        INVALID_ARGS = 2
        TIMEOUT = 255
    
    ## Representation of motor matrix using nested lists
    class MotorMatrix:
        def __init__(self):
            self.__data: List[List[float]] = []
            for row in range(8):
                self.__data.append([])
                for _ in range(6):
                    self.__data[row].append(0.0)
        
        ## Set a row of the motor matrix
        #  @param tnum Thruster number to set row for
        #  @param data Row data for the given thruster. 6 element float list [x y z pitch roll yaw]
        def set_row(self, tnum: int, data: List[float]):
            if len(data) != 6:
                return
            if tnum > 8 or tnum < 1:
                return
            for i in range(6):
                self.__data[tnum - 1][i] = float(data[i])
        
        ## Return motor matrix list representation
        def raw(self) -> List[List[float]]:
            # Python lists are passed around by reference and are mutable
            # Thus, make a copy, don't return the list directly
            copy_data: List[List[float]] = []
            for row in range(8):
                copy_data.append([])
                for col in range(6):
                    copy_data[row].append(self.__data[row][col])
            return copy_data

    ## Open communication with a control board
    #  @param port Serial port to communicate with control board by
    def __init__(self, port: str, debug = False):
        self.__last_wdog_feed = 0
        self.__read_thread = None
        self.__id_mutex = threading.Lock()
        self.__msg_id = 0
        self.__debug = debug
        self.__ser = serial.Serial(port, 115200)
        self.__ser.reset_input_buffer()
        self.__stop = False
        self.__ack_conds: Dict[int, threading.Condition] = {}
        self.__ack_errrs: Dict[int, int] = {}
        self.__read_thread = threading.Thread(target=self.__read_task, daemon=True)
        self.__read_thread.start()

    ## Cleanup on destruction
    def __del__(self):
        self.__stop = True
        if self.__read_thread is not None:
            self.__read_thread.join()
        try:
            self.__ser.close()
        except:
            pass

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

    ## Handle an acknowledge message
    #  @param msg_id ID of the message being acknowledged
    #  @param error_code Result of message being acknowledged
    def __handle_ack(self, msg_id: int, error_code: int):
        # Find a threading.Condition for the message being acknowledged
        # and set its result
        if msg_id in self.__ack_conds:
            with self.__ack_conds[msg_id]:
                self.__ack_errrs[msg_id] = error_code
                self.__ack_conds[msg_id].notify_all()

    ## Handle a message read from control board
    #  @param msg_id ID of the message
    #  @param msg Message read from control board
    def __handle_read_message(self, msg_id: int, msg: bytes):
        if self.__debug:
            print("Read: ({}) {}".format(msg_id, msg))
        
        if msg.startswith(b'ACK'):
            # Handle acknowledge messages
            # A, C, K, [id], [error_code]
            # [id] is a big endian id of the message being acknowledged (unsigned 16-bit int)
            # [error_code] is an unsigned 8-bit integer error code
            
            # Validate message length
            if len(msg) == 6:
                ack_id = struct.unpack(">H", msg[3:5])[0]
                err = msg[5]
                self.__handle_ack(ack_id, err)
        
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

            if self.__debug:
                print("RB: {}".format(b))

            # Parse the meaning of this byte
            if parse_escaped:
                # Currently escaped (previous byte was ESCAPE_BYTE)
                # Handle **valid** escape sequences (only special bytes can be escaped)
                # Ignore invalid sequences
                if b == START_BYTE or b == END_BYTE or b == ESCAPE_BYTE:
                    msg.extend(b)
                
                # Handled byte after escape byte. No longer escaped.
                parse_escaped = False
            elif parse_started:
                if b == START_BYTE:
                    # Handle start byte (special meaning when not escaped)
                    # Discard old data when start byte received
                    msg = bytearray()
                elif b == END_BYTE:
                    # Handle end byte (special meaning when not escaped)
                    # End byte means the buffer now holds the entire message
                    
                    # Calculate CRC of read data. Exclude last two bytes.
                    # Last two bytes are the CRC (big endian) appended to the original data
                    # First two bytes are message ID. These are INCLUDED in CRC calc.
                    calc_crc = self.__crc16_ccitt_false(bytes(msg[0:len(msg)-2]))
                    read_crc = struct.unpack(">H", msg[len(msg)-2:])[0]

                    if calc_crc == read_crc:
                        # This is a complete, valid message.
                        read_id = struct.unpack(">H", msg[0:2])[0]
                        self.__handle_read_message(read_id, bytes(msg[2:len(msg)-2]))
                    else:
                        # Got a complete message, but it is invalid. Ignore it.
                        if self.__debug:
                            print("Received message with invalid CRC!")
                        parse_started = False
                elif b == ESCAPE_BYTE:
                    # Handle escape byte (special meaning when not escaped)
                    parse_escaped = True
                else:
                    # handle normal bytes (these are just data)
                    msg.extend(b)
            elif b == START_BYTE:
                # Received a start byte. Start parsing. Discard old data.
                parse_started = True
                msg = bytearray()

    ## Prepare to send a message that will be acknowledged
    #  Must call before writing the message
    #  @param msg_id Id of message to be sent
    def __prepare_for_ack(self, msg_id: int):
        self.__ack_errrs[msg_id] = 0
        self.__ack_conds[msg_id] = threading.Condition()

    ## Wait to receive ack from control board
    #  @param msg_id ID of message to wait for ack
    #  @param timeout Time in seconds to wait for ack
    def __wait_for_ack(self, msg_id: int, timeout: float) -> AckError:
        ec = None
        with self.__ack_conds[msg_id]:
            if self.__ack_conds[msg_id].wait(timeout):
                ec = self.AckError(self.__ack_errrs[msg_id])
            else:
                ec = self.AckError.TIMEOUT
        del self.__ack_conds[msg_id]
        del self.__ack_errrs[msg_id]
        return ec

    ## Write one byte via serial
    #  @param b Single byte to write
    def __write_one(self, b: bytes):
        self.__ser.write(b)
        if self.__debug:
            print("WB: {}".format(b))

    ## Send a message to control board (properly encoded)
    #  @param msg Raw message (payload bytes) to send
    #  @param ack True if message needs to wait for ack (will setup structure to allow wait for ack)
    def __write_msg(self, msg: bytes, ack: bool = False):
        global START_BYTE, END_BYTE, ESCAPE_BYTE    

        # Generate the ID for this message and increment the global ID counter
        msg_id = -1
        with self.__id_mutex:
            msg_id = self.__msg_id
            self.__msg_id += 1
            if self.__msg_id > 65535:
                self.__msg_id = 0

        # Prepare ack structures if message will need ack
        # This must be done before message is sent to ensure ack handled properly
        if ack:
            self.__prepare_for_ack(msg_id)

        if self.__debug:
            print("WRITE: ({}) {}".format(msg_id, msg))

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
        high_byte = ((crc >> 8) & 0xFF).to_bytes(1, 'little')
        low_byte = (crc & 0xFF).to_bytes(1, 'little')
        if high_byte == START_BYTE or high_byte == END_BYTE or high_byte == ESCAPE_BYTE:
            self.__write_one(ESCAPE_BYTE)
        self.__write_one(high_byte)
        if low_byte == START_BYTE or low_byte == END_BYTE or low_byte == ESCAPE_BYTE:
            self.__write_one(ESCAPE_BYTE)
        self.__write_one(low_byte)

        # Write end byte
        self.__write_one(END_BYTE)

        return msg_id

    ## Set the motor matrix defining the vehicle's thruster configuration
    #  @param matrix Motor matrix object containing configuration to set
    def set_motor_matrix(self, matrix: MotorMatrix, timeout: float = 0.1) -> AckError:
        # Set each row one at a time
        raw_data = matrix.raw()
        for i in range(8):
            # Create row set message
            msg = bytearray()
            msg.extend(b'MMATS')
            msg.append(i + 1)
            msg.extend(struct.pack("<f", raw_data[i][0]))
            msg.extend(struct.pack("<f", raw_data[i][1]))
            msg.extend(struct.pack("<f", raw_data[i][2]))
            msg.extend(struct.pack("<f", raw_data[i][3]))
            msg.extend(struct.pack("<f", raw_data[i][4]))
            msg.extend(struct.pack("<f", raw_data[i][5]))

            # Send the message and wait for acknowledgement
            msg_id = self.__write_msg(bytes(msg), True)
            ack = self.__wait_for_ack(msg_id, timeout)
            if ack != self.AckError.NONE:
                return ack

        # Send update command (tells control board that matrix changed; recalculates some things)
        msg = b'MMATU'
        msg_id = self.__write_msg(msg, True)
        return self.__wait_for_ack(msg_id, timeout)

    ## Set thruster speeds in RAW mode
    #  @param speeds List of 8 speeds to send to control board. Must range from -1 to 1
    #  @return Error code (AckError enum) from control board (or timeout)
    def set_raw(self, speeds: List[float], timeout: float = 0.1) -> AckError:
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

        # Send the message and wait for acknowledgement
        msg_id = self.__write_msg(bytes(data), True)
        return self.__wait_for_ack(msg_id, timeout)

    ## Set thruster inversions (impacts all control modes)
    #  @param inversions List of 8 booleans indicating if thruster is inverted. 
    #                    True = inverted. False = not inverted.
    #  @return Error code (AckError enum) from control board (or timeout)
    def set_tinv(self, inversions: List[bool], timeout: float = 0.1) -> AckError:
        # Construct message to send
        data = bytearray()
        data.extend(b'TINV')
        inv_byte = 0
        for i in range(8):
            inv_byte <<= 1
            if inversions[7 - i]:
                inv_byte |= 1
        data.append(inv_byte)

        # Send the message and wait for acknowledgment
        msg_id = self.__write_msg(bytes(data), True)
        return self.__wait_for_ack(msg_id, timeout)

    ## Keep motors alive even when speed should not change
    #  If no speed set commands and no watchdog speed for long enough
    #  (1500ms at time of writing) then control board will kill motors
    #  Note: To avoid giving control board too much to process, feed commands
    #  are limited to every 350ms at most frequent
    def feed_motor_watchdog(self, timeout: float = 0.1) -> AckError:
        # Limit watchdog feed rate
        if time.time() - self.__last_wdog_feed < 0.35:
            return self.AckError.NONE
        
        # Send command to feed watchdog and wait for ack
        msg_id = self.__write_msg(b'WDGF', True)
        return self.__wait_for_ack(msg_id, timeout)

