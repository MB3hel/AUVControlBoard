#
# Copyright 2022 Marcus Behel
# 
# This file is part of AUVControlBoard-InterfaceScripts.
#
# AUVControlBoard-InterfaceScripts is free software: you can redistribute it and/or modify it under the terms of the GNU 
# General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your 
# option) any later version.
#
# AUVControlBoard-InterfaceScripts is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
# even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
# for more details.
#
# You should have received a copy of the GNU General Public License along with AUVControlBoard-InterfaceScripts. If not, 
# see <https://www.gnu.org/licenses/>. 
#

import serial
import copy
import struct
import time
import math
from enum import IntEnum
import threading
from typing import List, Dict, Tuple


START_BYTE = b'\xfd'
END_BYTE = b'\xfe'
ESCAPE_BYTE = b'\xff'


class ControlBoard:

    class AckError(IntEnum):
        NONE = 0
        UNKNOWN_MSG = 1
        INVALID_ARGS = 2
        INVALID_CMD = 3
        TIMEOUT = 255

    class BNO055Axis(IntEnum):
        P0 = 0
        P1 = 1
        P2 = 2
        P3 = 3
        P4 = 4
        P5 = 5
        P6 = 6
        P7 = 7
    
    class BNO055Data:
        def __init__(self):
            self.grav_x: float = 0.0
            self.grav_y: float = 0.0
            self.grav_z: float = 0.0
            self.quat_w: float = 0.0
            self.quat_x: float = 0.0
            self.quat_y: float = 0.0
            self.quat_z: float = 0.0
            self.euler_pitch: float = 0.0
            self.euler_roll: float = 0.0
            self.euler_yaw: float = 0.0
            self.accum_pitch: float = 0.0
            self.accum_roll: float = 0.0
            self.accum_yaw: float = 0.0
    
    class MS5837Data:
        def __init__(self):
            self.depth: float = 0.0

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
        self.__bno055_data = self.BNO055Data()
        self.__ms5837_data = self.MS5837Data()
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
        self.__ack_results: Dict[int, bytes] = {}
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
    def __handle_ack(self, msg_id: int, error_code: int, result: bytes):
        # Find a threading.Condition for the message being acknowledged
        # and set its result
        if msg_id in self.__ack_conds:
            with self.__ack_conds[msg_id]:
                self.__ack_errrs[msg_id] = error_code
                self.__ack_results[msg_id] = result
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
            if len(msg) >= 6:
                ack_id = struct.unpack(">H", msg[3:5])[0]
                err = msg[5]
                if len(msg) > 6:
                    result = msg[6:]
                else:
                    result = b''
                self.__handle_ack(ack_id, err, result)
        elif msg.startswith(b'WDGS'):
            # Motor watchdog status message
            # W, D, G, F, [status]
            # [status]: 1 = enabled, 0 = killed
            if self.__debug and len(msg) == 5:
                if msg[4] == 1:
                    print("Motors (re)enabled.")
                else:
                    print("Watchdog killed motors.")
        elif msg.startswith(b'BNO055D'):
            # BNO055 data status message
            if len(msg) == 47:
                self.__bno055_parse(msg[7:])
        elif msg.startswith(b'MS5837D'):
            # MS5837 data status message
            if len(msg) == 11:
                self.__ms5837_parse(msg[7:])
   
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

            # if self.__debug:
            #     print("RB: {}".format(b))

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
        self.__ack_results[msg_id] = b''
        self.__ack_conds[msg_id] = threading.Condition()

    ## Wait to receive ack from control board
    #  @param msg_id ID of message to wait for ack
    #  @param timeout Time in seconds to wait for ack
    def __wait_for_ack(self, msg_id: int, timeout: float) -> Tuple[AckError, bytes]:
        ec = None
        res = None
        with self.__ack_conds[msg_id]:
            if self.__ack_conds[msg_id].wait(timeout):
                ec = self.AckError(self.__ack_errrs[msg_id])
                res = self.__ack_results[msg_id]
            else:
                ec = self.AckError.TIMEOUT
                res = b''
        del self.__ack_conds[msg_id]
        del self.__ack_errrs[msg_id]
        del self.__ack_results[msg_id]
        return ec, res

    ## Write one byte via serial
    #  @param b Single byte to write
    def __write_one(self, b: bytes):
        # if self.__debug:
        #     print("WB: {}".format(b))
        self.__ser.write(b)

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
            ack, _ = self.__wait_for_ack(msg_id, timeout)
            if ack != self.AckError.NONE:
                return ack

        # Send update command (tells control board that matrix changed; recalculates some things)
        msg = b'MMATU'
        msg_id = self.__write_msg(msg, True)
        ack, _ = self.__wait_for_ack(msg_id, timeout)
        return ack

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
        ack, _ = self.__wait_for_ack(msg_id, timeout)
        return ack

    ## Set axis configuration for BNO055 IMU
    #  @param axis Axis configuration (see BNO055 datasheet) P0-P7 (BNO055Axis enum)
    #  @return Error code (AckError enum) from control board (or timeout)
    def set_bno055_axis(self, axis: BNO055Axis, timeout: float = 0.1) -> AckError:
        msg = bytearray()
        msg.extend(b'BNO055A')
        msg.append(int(axis))

        msg_id = self.__write_msg(bytes(msg), True)
        ack, _ = self.__wait_for_ack(msg_id, timeout)
        return ack



    ## Get sensor status (all sensors)
    #  @return Tuple[AckError, bool, bool]  error, bno055_ready, ms5837_ready
    def get_sensor_status(self, timeout: float = 0.1) -> Tuple[AckError, bool, bool]:
        # Send the message and wait for acknowledgement
        msg_id = self.__write_msg(b'SSTAT', True)
        ack, res = self.__wait_for_ack(msg_id, timeout)
        if ack != self.AckError.NONE:
            return ack, False, False
        bno055_ready = (res[0] & 0b00000001) != 0
        ms5837_ready = (res[0] & 0b00000010) != 0
        return ack, bno055_ready, ms5837_ready

    ## Parse byte data from BNO055 readings into the data class object
    def __bno055_parse(self, data: bytes):
        new_data = self.BNO055Data()
        
        # Parse data
        new_data.grav_x = struct.unpack("<f", data[0:4])[0]
        new_data.grav_y = struct.unpack("<f", data[4:8])[0]
        new_data.grav_z = struct.unpack("<f", data[8:12])[0]
        new_data.quat_w = struct.unpack("<f", data[12:16])[0]
        new_data.quat_x = struct.unpack("<f", data[16:20])[0]
        new_data.quat_y = struct.unpack("<f", data[20:24])[0]
        new_data.quat_z = struct.unpack("<f", data[24:28])[0]
        new_data.accum_pitch = struct.unpack("<f", data[28:32])[0]
        new_data.accum_roll = struct.unpack("<f", data[32:36])[0]
        new_data.accum_yaw = struct.unpack("<f", data[36:40])[0]

        # Calculate euler angles (ZYX) from quaternion
        t0 = +2.0 * (new_data.quat_w * new_data.quat_x + new_data.quat_y * new_data.quat_z)
        t1 = +1.0 - 2.0 * (new_data.quat_x * new_data.quat_x + new_data.quat_y * new_data.quat_y)
        new_data.euler_pitch = math.atan2(t0, t1)
        t2 = +2.0 * (new_data.quat_w * new_data.quat_y - new_data.quat_z * new_data.quat_x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        new_data.euler_roll = math.asin(t2)
        t3 = +2.0 * (new_data.quat_w * new_data.quat_z + new_data.quat_x * new_data.quat_y)
        t4 = +1.0 - 2.0 * (new_data.quat_y * new_data.quat_y + new_data.quat_z * new_data.quat_z)
        new_data.euler_yaw = math.atan2(t3, t4)
        new_data.euler_pitch *= 180.0 / math.pi
        new_data.euler_roll *= 180.0 / math.pi
        new_data.euler_yaw *= 180.0 / math.pi

        self.__bno055_data = new_data

    ## Read current BNO055 data. This is a single read. Does not start periodic reads
    #  Use get_bno055_data to get the last read data (either from this or a periodic read)
    #  @return AckError
    def read_bno055_once(self, timeout: float = 0.1) -> AckError:
        msg_id = self.__write_msg(b'BNO055R', True)
        ack, res = self.__wait_for_ack(msg_id, timeout)
        if ack != self.AckError.NONE:
            return ack
        self.__bno055_parse(res)
        return ack

    ## Enable / disable periodic status messages with BNO055 data. The periodically
    #  received data will be stored and is retrievable with get_bno055_data
    #  @param enable True to enable periodic reads. False to disable.
    #  @return AckError indicating success of processing this command.
    def read_bno055_periodic(self, enable: bool, timeout: float = 0.1) -> AckError:
        msg = bytearray()
        msg.extend(b'BNO055P')
        msg.append(1 if enable else 0)
        msg_id = self.__write_msg(bytes(msg), True)
        ack, res = self.__wait_for_ack(msg_id, timeout)
        return ack

    ## Get current BNO055 data. Current data is latest of either periodically received
    #  status messages or data received from a read_bno055_once call.
    #  @return BNO055Data object containing latest data
    def get_bno055_data(self) -> BNO055Data:
        return copy.copy(self.__bno055_data)

    ## Parse byte data from BNO055 readings into the data class object
    def __ms5837_parse(self, data: bytes):
        new_data = self.MS5837Data()
        new_data.depth = struct.unpack("<f", data[0:4])[0]
        self.__ms5837_data = new_data

    ## Read current MS5837 data. This is a single read. Does not start periodic reads
    #  Use get_ms5837_data to get the last read data (either from this or a periodic read)
    #  @return AckError
    def read_ms5837_once(self, timeout: float = 0.1) -> AckError:
        msg_id = self.__write_msg(b'MS5837R', True)
        ack, res = self.__wait_for_ack(msg_id, timeout)
        if ack != self.AckError.NONE:
            return ack
        self.__ms5837_parse(res)
        return ack

    ## Enable / disable periodic status messages with MS5837 data. The periodically
    #  received data will be stored and is retrievable with get_ms5837_data
    #  @param enable True to enable periodic reads. False to disable.
    #  @return AckError indicating success of processing this command.
    def read_ms5837_periodic(self, enable: bool, timeout: float = 0.1) -> AckError:
        msg = bytearray()
        msg.extend(b'MS5837P')
        msg.append(1 if enable else 0)
        msg_id = self.__write_msg(bytes(msg), True)
        ack, res = self.__wait_for_ack(msg_id, timeout)
        return ack

    ## Get current MS5837 data. Current data is latest of either periodically received
    #  status messages or data received from a read_ms5837_once call.
    #  @return MS5837Data object containing latest data
    def get_ms5837_data(self) -> MS5837Data:
        return copy.copy(self.__ms5837_data)



    ## Tune STABILITY_ASSIST pitch hold PID
    #  @param kp Proportional gain
    #  @param ki Integral gain
    #  @param kd Derivative gain
    #  @param kf Feed-forward gain
    #  @param limit Max output of PID (controls max speed in sassist mode)
    #  @return AckError
    def tune_sassist_pitch(self, kp: float, ki: float, kd: float, kf: float, limit: float, timeout: int) -> AckError:
        msg = bytearray()
        limit = abs(limit)
        if limit > 1.0:
            limit = 1.0
        msg.extend(b'SASSISTTNP')
        msg.extend(struct.pack("<f", kp))
        msg.extend(struct.pack("<f", ki))
        msg.extend(struct.pack("<f", kd))
        msg.extend(struct.pack("<f", kf))
        msg.extend(struct.pack("<f", limit))
        msg_id = self.__write_msg(bytes(msg), True)
        ack, _ = self.__wait_for_ack(msg_id, timeout)
        return ack

    ## Tune STABILITY_ASSIST roll hold PID
    #  @param kp Proportional gain
    #  @param ki Integral gain
    #  @param kd Derivative gain
    #  @param kf Feed-forward gain
    #  @param limit Max output of PID (controls max speed in sassist mode)
    #  @return AckError
    def tune_sassist_roll(self, kp: float, ki: float, kd: float, kf: float, limit: float, timeout: int) -> AckError:
        msg = bytearray()
        limit = abs(limit)
        if limit > 1.0:
            limit = 1.0
        msg.extend(b'SASSISTTNR')
        msg.extend(struct.pack("<f", kp))
        msg.extend(struct.pack("<f", ki))
        msg.extend(struct.pack("<f", kd))
        msg.extend(struct.pack("<f", kf))
        msg.extend(struct.pack("<f", limit))
        msg_id = self.__write_msg(bytes(msg), True)
        ack, _ = self.__wait_for_ack(msg_id, timeout)
        return ack

    ## Tune STABILITY_ASSIST yaw hold PID
    #  @param kp Proportional gain
    #  @param ki Integral gain
    #  @param kd Derivative gain
    #  @param kf Feed-forward gain
    #  @param limit Max output of PID (controls max speed in sassist mode)
    #  @return AckError
    def tune_sassist_yaw(self, kp: float, ki: float, kd: float, kf: float, limit: float, timeout: int) -> AckError:
        msg = bytearray()
        limit = abs(limit)
        if limit > 1.0:
            limit = 1.0
        msg.extend(b'SASSISTTNY')
        msg.extend(struct.pack("<f", kp))
        msg.extend(struct.pack("<f", ki))
        msg.extend(struct.pack("<f", kd))
        msg.extend(struct.pack("<f", kf))
        msg.extend(struct.pack("<f", limit))
        msg_id = self.__write_msg(bytes(msg), True)
        ack, _ = self.__wait_for_ack(msg_id, timeout)
        return ack

    ## Tune STABILITY_ASSIST depth hold PID
    #  @param kp Proportional gain
    #  @param ki Integral gain
    #  @param kd Derivative gain
    #  @param kf Feed-forward gain
    #  @param limit Max output of PID (controls max speed in sassist mode)
    #  @return AckError
    def tune_sassist_depth(self, kp: float, ki: float, kd: float, kf: float, limit: float, timeout: int) -> AckError:
        msg = bytearray()
        limit = abs(limit)
        if limit > 1.0:
            limit = 1.0
        msg.extend(b'SASSISTTND')
        msg.extend(struct.pack("<f", kp))
        msg.extend(struct.pack("<f", ki))
        msg.extend(struct.pack("<f", kd))
        msg.extend(struct.pack("<f", kf))
        msg.extend(struct.pack("<f", limit))
        msg_id = self.__write_msg(bytes(msg), True)
        ack, _ = self.__wait_for_ack(msg_id, timeout)
        return ack



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
        ack, _ = self.__wait_for_ack(msg_id, timeout)
        return ack

    ## Set thruster speeds in LOCAL mode
    #  All speeds are relative to robot (not world)
    #  @param x Speed in +x translation DoF (-1.0 to +1.0)
    #  @param y Speed in +y translation DoF (-1.0 to +1.0)
    #  @param z Speed in +z translation DoF (-1.0 to +1.0)
    #  @param pitch Speed in +pitch translation DoF (-1.0 to +1.0)
    #  @param roll Speed in +roll translation DoF (-1.0 to +1.0)
    #  @param yaw Speed in +yaw translation DoF (-1.0 to +1.0)
    def set_local(self, x: float, y: float, z: float, pitch: float, roll: float, yaw: float, timeout: float = 0.1) -> AckError:
        # Ensure provided data in valid range
        def limit(v: float):
            if v > 1.0:
                return 1.0
            if v < -1.0:
                return -1.0
            return v
        x = limit(x)
        y = limit(y)
        z = limit(z)
        pitch = limit(pitch)
        roll = limit(roll)
        yaw = limit(yaw)

        # Construct message to send
        data = bytearray()
        data.extend(b'LOCAL')
        data.extend(struct.pack("<f", x))
        data.extend(struct.pack("<f", y))
        data.extend(struct.pack("<f", z))
        data.extend(struct.pack("<f", pitch))
        data.extend(struct.pack("<f", roll))
        data.extend(struct.pack("<f", yaw))

        # Send the message and wait for acknowledgement
        msg_id = self.__write_msg(bytes(data), True)
        ack, _ = self.__wait_for_ack(msg_id, timeout)
        return ack

    ## Set thruster speeds in GLOBAL mode
    #  All speeds are relative to world (pitch and roll adjusted; NOT yaw adjusted)
    #  @param x Speed in +x translation DoF (-1.0 to +1.0)
    #  @param y Speed in +y translation DoF (-1.0 to +1.0)
    #  @param z Speed in +z translation DoF (-1.0 to +1.0)
    #  @param pitch Speed in +pitch translation DoF (-1.0 to +1.0)
    #  @param roll Speed in +roll translation DoF (-1.0 to +1.0)
    #  @param yaw Speed in +yaw translation DoF (-1.0 to +1.0)
    def set_global(self, x: float, y: float, z: float, pitch: float, roll: float, yaw: float, timeout: float = 0.1) -> AckError:
        # Ensure provided data in valid range
        def limit(v: float):
            if v > 1.0:
                return 1.0
            if v < -1.0:
                return -1.0
            return v
        x = limit(x)
        y = limit(y)
        z = limit(z)
        pitch = limit(pitch)
        roll = limit(roll)
        yaw = limit(yaw)

        # Construct message to send
        data = bytearray()
        data.extend(b'GLOBAL')
        data.extend(struct.pack("<f", x))
        data.extend(struct.pack("<f", y))
        data.extend(struct.pack("<f", z))
        data.extend(struct.pack("<f", pitch))
        data.extend(struct.pack("<f", roll))
        data.extend(struct.pack("<f", yaw))

        # Send the message and wait for acknowledgement
        msg_id = self.__write_msg(bytes(data), True)
        ack, _ = self.__wait_for_ack(msg_id, timeout)
        return ack

    ## Set thruster speeds in STABILITY_ASSIST mode (variant 1)
    #  @param x Speed in +x translation DoF (world relative; same as global mode)
    #  @param y Speed in +y translation DoF (world relative; same as global mode)
    #  @param yaw Speed in +yaw translation DoF (world relative; same as global mode)
    #  @param target_pitch Target pitch in degrees
    #  @param target_roll Target roll in degrees
    #  @param target_depth Target depth in meters (negative for below surface)
    def set_sassist1(self, x: float, y: float, yaw: float, target_pitch: float, target_roll: float, target_depth: float, timeout: float) -> AckError:
        def limit(v: float):
            if v > 1.0:
                return 1.0
            if v < -1.0:
                return -1.0
            return v
        x = limit(x)
        y = limit(y)
        yaw = limit(yaw)
        
        # Construct message to send
        data = bytearray()
        data.extend(b'SASSISTST1')
        data.extend(struct.pack("<f", x))
        data.extend(struct.pack("<f", y))
        data.extend(struct.pack("<f", yaw))
        data.extend(struct.pack("<f", target_pitch))
        data.extend(struct.pack("<f", target_roll))
        data.extend(struct.pack("<f", target_depth))

        # Send the message and wait for acknowledgement
        msg_id = self.__write_msg(bytes(data), True)
        ack, _ = self.__wait_for_ack(msg_id, timeout)
        return ack
    
    ## Set thruster speeds in STABILITY_ASSIST mode (variant 2)
    #  @param x Speed in +x translation DoF (world relative; same as global mode)
    #  @param y Speed in +y translation DoF (world relative; same as global mode)
    #  @param target_pitch Target pitch in degrees
    #  @param target_roll Target roll in degrees
    #  @param target_yaw Target yaw in degrees
    #  @param target_depth Target depth in meters (negative for below surface)
    def set_sassist2(self, x: float, y: float, target_pitch: float, target_roll: float, target_yaw: float, target_depth: float, timeout: float) -> AckError:
        def limit(v: float):
            if v > 1.0:
                return 1.0
            if v < -1.0:
                return -1.0
            return v
        x = limit(x)
        y = limit(y)
        
        # Construct message to send
        data = bytearray()
        data.extend(b'SASSISTST2')
        data.extend(struct.pack("<f", x))
        data.extend(struct.pack("<f", y))
        data.extend(struct.pack("<f", target_pitch))
        data.extend(struct.pack("<f", target_roll))
        data.extend(struct.pack("<f", target_yaw))
        data.extend(struct.pack("<f", target_depth))

        # Send the message and wait for acknowledgement
        msg_id = self.__write_msg(bytes(data), True)
        ack, _ = self.__wait_for_ack(msg_id, timeout)
        return ack

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
        ack, _ = self.__wait_for_ack(msg_id, timeout)
        return ack

