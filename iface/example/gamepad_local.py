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
# Drive the vehicle using a gamepad. Controls use LOCAL mode.
# Requires https://github.com/ArPiRobot/ArPiRobot-DriveStation
################################################################################
# Author: Marcus Behel
# Date: June 11, 2023
# Version: 1.0.0
################################################################################

if __name__ == "__main__":
    print("Do not run this script directly. Use launch.py to run it.")
    exit(1)

import signal
from arp_gamepad import Gamepad, NetMgr
import time
from control_board import ControlBoard, Simulator


def run(cb: ControlBoard, s: Simulator) -> int:
    print("Starting networking")
    netmgr = NetMgr()
    gp0 = Gamepad.get(0)
    
    print("Enter main loop")
    try:
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
            pitch *= 0.7
            roll *= 0.2
            z *= 0.5

            # Set the speeds
            # Note that speed updates feed motor watchdog
            # This happens often enough that it is unnecessary to explicitly feed watchdog
            cb.set_local(x, y, z, pitch, roll, yaw)
            time.sleep(0.02)
    except (KeyboardInterrupt, Exception) as e:
        cb.set_local(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        old_handler = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, dummy_handler)
        del netmgr
        signal.signal(signal.SIGINT, old_handler)
        if not isinstance(e, KeyboardInterrupt):
            raise e


def dummy_handler(signum, frame):
    # Used temporarily suppress ctrl-c
    pass
