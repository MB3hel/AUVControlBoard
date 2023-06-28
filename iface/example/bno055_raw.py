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
# Print raw IMU (BNO055) data
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
import platform
import os


def run(cb: ControlBoard, s: Simulator) -> int:
    ############################################################################
    # Query sensor status
    ############################################################################
    print("Query sensor status...", end="")
    res, bno055, ms5837 = cb.get_sensor_status()
    if res != ControlBoard.AckError.NONE:
        print("Fail.")
        return 1
    print("Done.")
    print("BNO055: {}".format("Ready" if bno055 else "Not Ready"))
    print()

    time.sleep(1)

    ############################################################################
    # Periodically print sensor data
    ############################################################################
    failures = 0
    while True:
        ack, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = cb.read_bno055_raw()
        if ack == cb.AckError.NONE:
            failures = 0
            if platform.system() == "Windows":
                os.system("cls")
            else:
                os.system("clear")

            print("Accelerometer Data:")
            print("  X: {0}".format(accel_x))
            print("  Y: {0}".format(accel_y))
            print("  Z: {0}".format(accel_z))
            print("Gyroscope Data:")
            print("  X: {0}".format(gyro_x))
            print("  Y: {0}".format(gyro_y))
            print("  Z: {0}".format(gyro_z))
            print("Press CTRL+C to exit")
        else:
            failures += 1
            if failures == 5:
                print("Failed to read raw sensor data!")
                return 1

        time.sleep(0.1)