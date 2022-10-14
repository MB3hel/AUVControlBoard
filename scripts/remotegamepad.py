#!/usr/bin/env python3
# Intended to run on a SBC on the robot with control board connected
# Receives gamepad data over UDP (using protocol of ArPiRobot-DriveStation)
# Moves using vectored motion based on gamepad data

import math
import struct
import threading
import traceback
from typing import Dict, List
import time
from socketserver import TCPServer, UDPServer, BaseRequestHandler, BaseServer
from controlboard import ControlBoard


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
    class ControllerHandler(BaseRequestHandler):
        def handle(self):
            self.data = self.request[0]
            cnum = self.data[0]
            g = Gamepad.get(cnum)
            try:
                g.update_data(self.data)
            except:
                traceback.print_exc()
                # Not enough data, parsing failed
                print("PARSE FAIL")
                pass


    class NullHandler(BaseRequestHandler):
        def handle(self):
            # Keep connection open until client closes it
            while True:
                self.data = self.request.recv(1024)
                if not self.data:
                    break

    def __init__(self):
        # Must match all ports used by ArPiRobot-CoreLib
        # So drive station can properly connect
        self.__crludp = UDPServer(("0.0.0.0", 8090), NetMgr.ControllerHandler)
        self.__cmdtcp = TCPServer(("0.0.0.0", 8091), NetMgr.NullHandler)
        self.__ntbtcp = TCPServer(("0.0.0.0", 8092), NetMgr.NullHandler)
        self.__logtcp = TCPServer(("0.0.0.0", 8093), NetMgr.NullHandler)
        self.__crlthd = threading.Thread(target=self.__run_server, args=(self.__crludp,), daemon=True)
        self.__cmdthd = threading.Thread(target=self.__run_server, args=(self.__cmdtcp,), daemon=True)
        self.__ntbthd = threading.Thread(target=self.__run_server, args=(self.__ntbtcp,), daemon=True)
        self.__logthd = threading.Thread(target=self.__run_server, args=(self.__logtcp,), daemon=True)
        self.__crlthd.start()
        self.__cmdthd.start()
        self.__ntbthd.start()
        self.__logthd.start()
    
    def __del__(self):
        self.__crludp.shutdown()
        self.__cmdtcp.shutdown()
        self.__ntbtcp.shutdown()
        self.__logtcp.shutdown()
        self.__crlthd.join()
        self.__cmdthd.join()
        self.__ntbthd.join()
        self.__logthd.join()

    def __run_server(self, server: BaseServer):
        server.serve_forever()


if __name__ == "__main__":
    try:
        print("Opening communication with control board...", end="")
        try:
            cb = ControlBoard("/dev/ttyACM0")
            print("Done.")
        except:
            print("Fail.")
            exit(1)
        
        print("Setting control board to LOCAL mode...", end="")
        if cb.set_mode(ControlBoard.Mode.LOCAL):
            print("Done.")
        else:
            print("Fail.")
            exit(1)

        print("Setting thruster inversions...", end="")
        if cb.set_inverted(True, True, False, False, True, False, False, True):
            print("Done.")
        else:
            print("Fail.")
            exit(1)


        print("Initializing networking...", end="")
        try:
            netmgr = NetMgr()
            print("Done.")
        except:
            print("Fail.")
            exit(1)
        
        print("Begin main loop.")
        gp0 = Gamepad.get(0)
        while True:
            # Left stick x and y = strafe
            # Left trigger = yaw ccw
            # Right trigger = yaw cw
            # Dpad up / down = surface / submerge
            # Right stick x = roll
            # Right stick y = pitch
            x = gp0.get_axis(0, 0.1)
            y = gp0.get_axis(1, 0.1)
            roll = gp0.get_axis(2, 0.1)
            pitch = gp0.get_axis(3, 0.1)
            yawccw = gp0.get_axis(4, 0.1)
            yawcw = gp0.get_axis(5, 0.1)
            yaw = (yawccw - yawcw)
            dpad = gp0.get_dpad(0)
            if dpad == 8 or dpad == 1 or dpad == 2:
                z = 1.0
            elif dpad == 6 or dpad == 5 or dpad == 4:
                z = -1.0
            else:
                z = 0.0

            # Scale speeds to make the robot more controllable
            x *= 0.5
            y *= -0.5
            yaw *= 0.45
            pitch *= 0.3
            roll *= 0.2
            z *= 0.5

            # Set the speeds
            cb.set_local(x, y, z, pitch, roll, yaw)
            time.sleep(0.02)
    except KeyboardInterrupt:
        cb.set_local(0, 0, 0, 0, 0 ,0)
        exit(0)
