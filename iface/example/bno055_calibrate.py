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


def prompt_yn(msg: str, default = "") -> bool:
    if default == "y" or default == "Y":
        defstr = "Y/n"
    elif default == "n" or default == "N":
        defstr = "y/N"
    else:
        defstr = "y/n"
    prmpt = "{0} ({1}): ".format(msg, defstr)
    while True:
        res = input(prmpt)
        if res == "Y" or res == "y":
            return True
        elif res == "N" or res == "n":
            return False
        elif res == "":
            if default == "y" or default == "Y":
                return True
            elif default == "n" or default == "N":
                return False


def run(cb: ControlBoard, s: Simulator) -> int:
    if platform.system() == "Windows":
        os.system("cls")
    else:
        os.system("clear")

    print("BNO055 IMU Calibration")
    print("================================================================================")

    ack, sc_valid, sc_cal = cb.read_stored_bno055_calibration()
    if ack != cb.AckError.NONE:
        print("Failed to read currently stored calibration.")
        return 1

    if sc_valid:
        print("  There is currently a calibration for the BNO055 stored on the control board.")
        print("  Stored Calibration Values:")
        print("    Accelerometer Offset X: {0}".format(sc_cal.accel_offset_x))
        print("    Accelerometer Offset Y: {0}".format(sc_cal.accel_offset_y))
        print("    Accelerometer Offset Z: {0}".format(sc_cal.accel_offset_z))
        print("    Accelerometer Radius  : {0}".format(sc_cal.accel_radius))
        print("    Gyroscope Offset X    : {0}".format(sc_cal.gyro_offset_x))
        print("    Gyroscope Offset Y    : {0}".format(sc_cal.gyro_offset_y))
        print("    Gyroscope Offset Z    : {0}".format(sc_cal.gyro_offset_z))
        print("  This calibration must be erased to re-calibrate.")
        print("")
        res = prompt_yn("  Erase stored calibration?", "Y")
        if res:
            print("  Erasing calibration and resetting IMU...", end="")
            if cb.erase_stored_bno055_calibration() == cb.AckError.NONE:
                print("Done.")
            else:
                print("Fail.")
                return 1
            print("  Waiting 3 seconds for IMU startup after resetting.")
            time.sleep(3)
        else:
            return 1

    print("")
    print("Calibration Procedure:")
    print("  This script will show the calibration status of various senors. A status of 3")
    print("  means that the sensor is fully calibrated. Follow the instructions below to")
    print("  achieve a status of 3 for each sensor. Once achieved, you can save the")
    print("  calibration to the control board for future use.")
    print("")
    print("  Gyroscope Calibration:")
    print("    - Place the device in a single stable position for a period of few seconds")
    print("      to allow the gyroscope to calibrate")
    print("  Accelerometer Calibration:")
    print("    - Place the device in 6 different stable positions for a period of few")
    print("      seconds to allow the accelerometer to calibrate.")
    print("    - Make sure that there is slow movement between 2 stable positions")
    print("    - The 6 stable positions could be in any direction, but make sure that the")
    print("      device is lying at least once perpendicular to the x, y and z axis.")
    print("")
    input("  Press enter to begin calibration.")
    print("")

    failures = 0
    while True:
        ack, status = cb.bno055_read_calibration_status()
        if ack != cb.AckError.NONE:
            failures += 1
            if failures == 3:
                print("")
                print("  Calibration failed. Communication with the sensor failed!")
                return 1
        else:
            failures = 0
            mag_stat = status & 0b11
            status >>= 2
            acc_stat = status & 0b11
            status >>= 2
            gyr_stat = status & 0b11
            status >>= 2
            sys_stat = status & 0b11
            # Note that system calibration only goes to 3 when both gyro and mag are 3
            print("  Calibration Status (3 = fully calibrated):")
            # print("  Magnetometer:  {0}".format(mag_stat))
            print("    Accelerometer: {0}".format(acc_stat))
            print("    Gyroscope:     {0}".format(gyr_stat))
            # print("  System:        {0}".format(sys_stat))
            if acc_stat == 3 and gyr_stat == 3:
                break
        time.sleep(0.5)

    print("")
    print("  Calibration successful!")
    while True:
        ack, cal = cb.bno055_read_calibration()
        if ack == cb.AckError.NONE:
            break
        failures += 1
        if failures == 3:
            print("  Failed to read calibration constants from sensor!")
            return 1
        time.sleep(0.5)
    print("")
    print("  Calibration Values:")
    print("    Accelerometer Offset X: {0}".format(cal.accel_offset_x))
    print("    Accelerometer Offset Y: {0}".format(cal.accel_offset_y))
    print("    Accelerometer Offset Z: {0}".format(cal.accel_offset_z))
    print("    Accelerometer Radius  : {0}".format(cal.accel_radius))
    print("    Gyroscope Offset X    : {0}".format(cal.gyro_offset_x))
    print("    Gyroscope Offset Y    : {0}".format(cal.gyro_offset_y))
    print("    Gyroscope Offset Z    : {0}".format(cal.gyro_offset_z))
    print("")
    print("  You may now save the current calibration to the control board. Doing so will")
    print("  result in the calibration being automatically applied at power on.")
    print("")
    if prompt_yn("  Save calibration to control board?", "N"):
        ack = cb.store_bno055_calibration(cal)
        if ack == cb.AckError.NONE:
            print("")
            print("  Calibration saved. It will be applied after control board power on.")
        else:
            print("")
            print("  Failed to save calibration!")
            return 1
    else:
        print("")
        print("  Calibration not saved. Calibration must be repeated after the next power cycle")
        print("  of the control board.")