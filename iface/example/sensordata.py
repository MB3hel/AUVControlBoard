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
# Print sensor status and data from the vehicle
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
    print("MS5837: {}".format("Ready" if ms5837 else "Not Ready"))
    print()

    time.sleep(2)

    ############################################################################
    # Setup
    ############################################################################
    print("Enable periodic sensor data...", end="")
    if cb.read_bno055_periodic(True) != ControlBoard.AckError.NONE:
        print("Fail.")
        return 1
    if cb.read_ms5837_periodic(True) != ControlBoard.AckError.NONE:
        print("Fail.")
        return 1
    print("Done.")
    

    ############################################################################
    # Periodically print sensor data
    ############################################################################
    while True:
        imu_data = cb.get_bno055_data()
        depth_data = cb.get_ms5837_data()
        
        if platform.system() == "Windows":
            os.system("cls")
        else:
            os.system("clear")
        
        print("Pitch: {}".format(imu_data.pitch))
        print("Roll: {}".format(imu_data.roll))
        print("Yaw: {}".format(imu_data.yaw))
        print()
        print("W: {}".format(imu_data.quat_w))
        print("X: {}".format(imu_data.quat_x))
        print("Y: {}".format(imu_data.quat_y))
        print("Z: {}".format(imu_data.quat_z))
        print()
        print("Accum Pitch: {}".format(imu_data.accum_pitch))
        print("Accum Roll: {}".format(imu_data.accum_roll))
        print("Accum Yaw: {}".format(imu_data.accum_yaw))
        print()
        print("Depth: {}".format(depth_data.depth))
        print("Pressure: {}".format(depth_data.pressure))
        print("Temperature: {}".format(depth_data.temperature))

        time.sleep(0.1)