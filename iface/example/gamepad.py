################################################################################
# Copyright 2022-2023 Marcus Behel
#
# This file is part of AUVControlBoard.
# 
# AUVControlBoard is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# AUVControlBoard is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public License
# along with AUVControlBoard.  If not, see <https://www.gnu.org/licenses/>.
################################################################################
# Drive the vehicle using a gamepad. Controls use STABILITY_ASSIST & 
#    ORIENTATION_HOLD modes.
# Requires https://github.com/ArPiRobot/ArPiRobot-DriveStation to send
#    controller data.
# NOTE: This script ignores the enabled / disabled states of the ArPiRobot
#       drive station!
################################################################################
# Author: Marcus Behel
# Date: June 11, 2023
# Version: 1.0.0
################################################################################

if __name__ == "__main__":
    print("Do not run this script directly. Use launch.py to run it.")
    exit(1)

import signal
import time
from control_board import ControlBoard, Simulator
import socket
from typing import Dict, List
import threading
import struct
import math
import traceback


########################################################################################################################
# Support code to interface with ArPiRobot Drive Station
# https://github.com/ArPiRobot/ArPiRobot-DriveStation
# This only supports gamepads. The network table and enable / disable commands
# are not supported.
########################################################################################################################
class Gamepad:
    __instances: Dict[int, 'Gamepad'] = {}
    __instance_lock = threading.Lock()

    @staticmethod
    def get(num: int):
        with Gamepad.__instance_lock:
            if num in Gamepad.__instances:
                return Gamepad.__instances[num]
            Gamepad.__instances[num] = Gamepad()
            return Gamepad.__instances[num]

    def __init__(self):
        self.__axes: List[float] = []
        self.__btns: List[bool] = []
        self.__dpads: List[int] = []
        self.__data_lock = threading.Lock()
        self.__update_time = 0
    
    def update_data(self, data: bytes):
        with self.__data_lock:
            num = data[0]
            axis_count = data[1]
            btn_count = data[2]
            dpad_count = data[3]
            if len(self.__axes) != axis_count:
                self.__axes.clear()
                self.__axes.extend([0.0] * axis_count)
            if len(self.__btns) != btn_count:
                self.__btns.clear()
                self.__btns.extend([False] * btn_count)
            if len(self.__dpads) != dpad_count:
                self.__dpads.clear()
                self.__dpads.extend([0] * dpad_count)
            
            # Each axis is signed 16-bit integer (full range)\
            offset = 4
            for i in range(axis_count):
                tmp = struct.unpack(">h", data[offset:offset+2])[0]
                if tmp <= 0:
                    self.__axes[i] = tmp / 32768.0
                else:
                    self.__axes[i] = tmp / 32767.0
                offset += 2

            # Each byte encodes 8 button states
            for i in range(math.ceil(btn_count / 8.0) - 1, -1, -1):
                b = data[4 + (2 * axis_count) + i]
                for j in range(7, -1, -1):
                    if(i * 8 + j) < btn_count:
                        self.__btns[i * 8 + j] = ( (b & 0x01) == 1)
                    b >>= 1
            
            # Each byte encodes 2 dpads (int between 0 and 8)
            for i in range(math.ceil(dpad_count / 2.0) - 1, -1, -1):
                b = data[4 + (2 * axis_count) + math.ceil(btn_count / 8.0) + i]
                for j in range(1, -1, -1):
                    if (i * 2 + j) < dpad_count:
                        self.__dpads[i * 2 + j] = (b & 0x0F)
                    b >>= 4

            self.__update_time = time.time()

    def get_axis(self, axis: int, deadband = 0) -> float:
        with self.__data_lock:
            if time.time() - self.__update_time > 1:
                return 0.0
            if axis < len(self.__axes):
                v = self.__axes[axis]
                if abs(v) < deadband:
                    return 0.0
                else:
                    return (v - (abs(v) / v * deadband)) / (1 - deadband)
            return 0.0
    
    def get_button(self, btn: int) -> bool:
        with self.__data_lock:
            if time.time() - self.__update_time > 1:
                return False
            if btn < len(self.__btns):
                return self.__btns[btn]
            return False
    
    def get_dpad(self, dpad: int) -> int:
        with self.__data_lock:
            if time.time() - self.__update_time > 1:
                return 0
            if dpad < len(self.__dpads):
                return self.__dpads[dpad]
            return 0


class NetMgr:
    def __init__(self):
        self.__controller_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.__cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__ntb_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__log_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__controller_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.__controller_sock.bind(("0.0.0.0", 8090))
        self.__cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.__cmd_sock.bind(("0.0.0.0", 8091))
        self.__cmd_sock.listen()
        self.__ntb_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.__ntb_sock.bind(("0.0.0.0", 8092))
        self.__ntb_sock.listen()
        self.__log_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.__log_sock.bind(("0.0.0.0", 8093))
        self.__log_sock.listen()
        self.__crl_t = threading.Thread(target=self.__run_controller, daemon=True)
        self.__cmd_t = threading.Thread(target=self.__run, daemon=True, args=(self.__cmd_sock,))
        self.__ntb_t = threading.Thread(target=self.__run, daemon=True, args=(self.__ntb_sock,))
        self.__log_t = threading.Thread(target=self.__run, daemon=True, args=(self.__log_sock,))
        self.__crl_t.start()
        self.__cmd_t.start()
        self.__ntb_t.start()
        self.__log_t.start()

    
    def __run(self, sock: socket.socket):
        clients = []
        while True:
            conn, _ = sock.accept()
            clients.append(conn)
    
    def __run_controller(self):
        while True:
            data, _ = self.__controller_sock.recvfrom(1024)
            cnum = data[0]
            g = Gamepad.get(cnum)
            try:
                g.update_data(data)
            except:
                traceback.print_exc()
                # Not enough data, parsing failed
                print("PARSE FAIL")
                pass
    
    def __del__(self):
        self.__cmd_sock.close()
        self.__ntb_sock.close()
        self.__log_sock.close()
        self.__controller_sock.close()

########################################################################################################################




def run(cb: ControlBoard, s: Simulator) -> int:
    print("Starting networking")
    netmgr = NetMgr()
    gp0 = Gamepad.get(0)

    print("Checking sensors")
    ack, bno055, ms5837 = cb.get_sensor_status()
    if ack != cb.AckError.NONE:
        print("Failed to check senor status!")
        return 1
    if not bno055:
        print("IMU not ready")
        return 1
    if not ms5837:
        print("Depth sensor not ready")
        return 1

    print("Enabling sensor data")
    # Force valid data now (when these functions return without error, cb has valid data for each sensor)
    cb.read_imu_once()
    cb.read_depth_once()
    # Then enable periodic reads for future data
    cb.read_imu_periodic(True)
    cb.read_depth_periodic(True)

    print("")
    print("********************************************************************************")
    print("IMPORTANT")
    print("  1. Enable / Disable in the drive station does nothing!")
    print("     The code is always enabled!")
    print("  2. Check the box next to the controller you want to use")
    print("     and drag it to the top (make it number 0).")
    print("********************************************************************************")
    print("")

    print("Enter main loop")
    depth_target = -0.2
    pitch_target = 0.0
    roll_target = 0.0
    was_ohold = False
    try:
        while True:
            # Left stick x and y = strafe
            # Left trigger = yaw ccw
            # Right trigger = yaw cw
            # Dpad up / down = surface / submerge
            x = gp0.get_axis(0, 0.1)
            y = gp0.get_axis(1, 0.1)
            z = 0.0
            yawccw = gp0.get_axis(4, 0.1)
            yawcw = gp0.get_axis(5, 0.1)
            yaw = (yawccw - yawcw)
            dpad = gp0.get_dpad(0)

            if dpad == 8 or dpad == 1 or dpad == 2:
                z = 1.0
            elif dpad == 6 or dpad == 5 or dpad == 4:
                z = -1.0
            

            if depth_target > -0.1:
                depth_target = -0.1

            # Scale speeds to make the robot more controllable
            x *= 0.6
            y *= -0.6
            yaw *= 0.6
            z *= 0.5

            # Set the speeds
            # Note that speed updates feed motor watchdog
            # This happens often enough that it is unnecessary to explicitly feed watchdog
            if z == 0.0:
                # No z speed from dpad, hold current depth (SASSIST mode)
                if was_ohold:
                    depth_target = cb.get_depth_data().depth
                cb.set_sassist1(x, y, yaw, pitch_target, roll_target, depth_target)
            else:
                # Use OHOLD mode to maintain orientation, but change depth based on dpad
                cb.set_ohold1(x, y, z, yaw, pitch_target, roll_target)
                was_ohold = True
            time.sleep(0.02)
    except (KeyboardInterrupt, Exception) as e:
        cb.set_global(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        old_handler = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, dummy_handler)
        del netmgr
        signal.signal(signal.SIGINT, old_handler)
        if not isinstance(e, KeyboardInterrupt):
            raise e


def dummy_handler(signum, frame):
    # Used temporarily suppress ctrl-c
    pass

