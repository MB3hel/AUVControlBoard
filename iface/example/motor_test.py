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
# Test motor operating using RAW mode. Interactive script which prompts the 
# user for test parameters (thruster speed and duration of motion) then
# allows the user to enter thrusters to run one at a time.
################################################################################
# Author: Marcus Behel
# Date: May 10, 2023
# Version: 1.0.0
################################################################################

if __name__ == "__main__":
    print("Do not run this script directly. Use launch.py to run it.")
    exit(1)

from control_board import ControlBoard, Simulator
import time


def run(cb: ControlBoard, s: Simulator) -> int:
    ############################################################################
    # Settings
    ############################################################################
    print()
    print("Input settings: ")
    speed = input("Speed (-1.0 to 1.0; default = 0.3): ")
    duration = input("Duration (sec; default = 1.0):")
    try:
        speed = float(speed)
        if speed > 1.0:
            speed = 1.0
        if speed < -1.0:
            speed = -1.0
    except:
        speed = 0.3
    try:
        duration = float(duration)
    except:
        duration = 1.0
    print()

    ############################################################################
    # Motor test
    ############################################################################
    print("Enter a thruster number 1-8 to run that thruster at the specified speed for the specified duration. Enter q to exit.")
    while True:
        res = input("Thruster: ")
        if res == "Q" or res == "q":
            break
        try:
            thr = int(res)

            if thr < 1 or thr > 8:
                raise Exception()
            idx = thr - 1

            # Set desired speed
            speeds = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            speeds[idx] = speed
            attempt = 0
            while cb.set_raw(speeds) != ControlBoard.AckError.NONE:
                attempt += 1
                if attempt > 5:
                    print("Failed to set thruster speeds.")
                    return 1
                time.sleep(0.02)
            
            # Wait for desired duration. Keep motors alive with watchdog feeds
            start = time.time()
            while time.time() - start < duration:
                cb.feed_motor_watchdog()
                time.sleep(0.1)

            # Stop the thruster
            speeds[idx] = 0.0
            attempt = 0
            while cb.set_raw(speeds) != ControlBoard.AckError.NONE:
                attempt += 1
                if attempt > 5:
                    print("Failed to set thruster speeds.")
                    return 1
                time.sleep(0.02)

        except:
            print("Invalid thruster number!")
    return 0
