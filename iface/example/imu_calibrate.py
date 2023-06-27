################################################################################
# Copyright 2023 Marcus Behel
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
# Read IMU calibration status and allow saving calibration constants
################################################################################
# Author: Marcus Behel
# Date: June 27, 2023
# Version: 1.0.0
################################################################################

import platform
import os

if __name__ == "__main__":
    print("Do not run this script directly. Use launch.py to run it.")
    exit(1)

from control_board import ControlBoard, Simulator
import time
import threading


def run(cb: ControlBoard, s: Simulator) -> int:
    failures = 0
    while True:
        ack, cal = cb.bno055_read_calibration()
        if ack != cb.AckError.NONE:
            failures += 1
            if failures == 3:
                print("Failed to read calibration data from control board!")
                return 1
        else:
            failures = 0

            if platform.system() == "Windows":
                os.system("cls")
            else:
                os.system("clear")

            status = cal.status
            mag_stat = status & 0b11
            status >>= 2
            acc_stat = status & 0b11
            status >>= 2
            gyr_stat = status & 0b11
            status >>= 2
            sys_stat = status & 0b11
            # Note that system calibration only goes to 3 when both gyro and mag are 3
            print("Calibration Status (3 = fully calibrated):")
            # print("  Magnetometer:  {0}".format(mag_stat))
            print("  Accelerometer: {0}".format(acc_stat))
            print("  Gyroscope:     {0}".format(gyr_stat))
            # print("  System:        {0}".format(sys_stat))
        time.sleep(0.5)