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
# Drive the vehicle using a gamepad. Controls use STABILITY_ASSIST mode.
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

    print("Enabling sensor data")
    cb.read_bno055_periodic(True)
    cb.read_ms5837_periodic(True)
    time.sleep(0.5) # Wait for data to actually be sent

    print("Enter main loop")
    depth_target = cb.get_ms5837_data().depth
    pitch_target = 0.0
    roll_target = 0.0
    try:
        while True:
            # Left stick x and y = strafe
            # Left trigger = yaw ccw
            # Right trigger = yaw cw
            # Dpad up / down = surface / submerge
            x = gp0.get_axis(0, 0.1)
            y = gp0.get_axis(1, 0.1)
            yawccw = gp0.get_axis(4, 0.1)
            yawcw = gp0.get_axis(5, 0.1)
            yaw = (yawccw - yawcw)
            dpad = gp0.get_dpad(0)

            if dpad == 8 or dpad == 1 or dpad == 2:
                depth_target = cb.get_ms5837_data().depth + 0.2
            elif dpad == 6 or dpad == 5 or dpad == 4:
                depth_target = cb.get_ms5837_data().depth - 0.2

            # Scale speeds to make the robot more controllable
            x *= 0.5
            y *= -0.5
            yaw *= 0.45

            # Set the speeds
            # Note that speed updates feed motor watchdog
            # This happens often enough that it is unnecessary to explicitly feed watchdog
            cb.set_sassist1(x, y, yaw, pitch_target, roll_target, depth_target)
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

