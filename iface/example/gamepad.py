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
from arp_gamepad import Gamepad, NetMgr
import time
from control_board import ControlBoard, Simulator


def run(cb: ControlBoard, s: Simulator) -> int:
    print("Starting networking")
    netmgr = NetMgr()
    gp0 = Gamepad.get(0)

    print("Checking sensors")
    ack, bno055, ms5837 = cb.get_sensor_status()
    if ack == cb.AckError.NONE:
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
    cb.read_bno055_once()
    cb.read_ms5837_once()
    # Then enable periodic reads for future data
    cb.read_bno055_periodic(True)
    cb.read_ms5837_periodic(True)

    print("Enter main loop")
    print("WARNING: Vehicle is ENABLED! Enable / Disable in the drive station does nothing here!")
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
            z = 0.0
            yawccw = gp0.get_axis(4, 0.1)
            yawcw = gp0.get_axis(5, 0.1)
            yaw = (yawccw - yawcw)
            dpad = gp0.get_dpad(0)

            if dpad == 8 or dpad == 1 or dpad == 2:
                z = 1.0
            elif dpad == 6 or dpad == 5 or dpad == 4:
                z = -1.0
            

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
                cb.set_sassist1(x, y, yaw, pitch_target, roll_target, depth_target)
            else:
                # Use OHOLD mode to maintain orientation, but change depth based on dpad
                cb.set_ohold1(x, y, z, yaw, pitch_target, roll_target)
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

