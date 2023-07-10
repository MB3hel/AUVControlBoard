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


def prompt_int(msg: str) -> int:
    while True:
        num_str = input(msg)
        try:
            num = int(num_str)
            return num
        except ValueError:
            pass


def check_existing_cal(cb: ControlBoard, s: Simulator) -> int:
    ack, sc_valid, sc_cal = cb.read_stored_bno055_calibration()
    if ack != cb.AckError.NONE:
        print("  Failed to read currently stored calibration.")
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
            print("  Erasing calibration...", end="")
            if cb.erase_stored_bno055_calibration() == cb.AckError.NONE:
                print("Done.")
            else:
                print("Fail.")
                return 1
            print("")
        else:
            return 1
    print("  There is no BNO055 calibration stored on the control board.")
    return 0


def guided_calibration(cb: ControlBoard, s: Simulator) -> int:
    # Show calibration instructions
    print("")
    print("Guided Calibration")
    print("--------------------------------------------------------------------------------")
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

    # This erases any existing live calibration constants (auto generated calibration on BNO055 itself)
    # Avoids issues where this script is run after the device has moved a lot
    # In this case, the BNO055 may have generated constants (bad ones likely)
    # So the status would be "3" immediately
    ack = cb.bno055_reset()
    if ack != cb.AckError.NONE:
        print("Communication failed while resetting BNO055.")
        return 1

    # Periodically show calibration status
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

    # Show resulting calibration values
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

    # Prompt to save calibration
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
    return 0


def manual_calibration(cb: ControlBoard, s: Simulator):
    print("")
    print("Manual Calibration")
    print("--------------------------------------------------------------------------------")
    print("  Enter values for each calibration constant. These are all signed 16-bit")
    print("  integers. The valid range and meaning of each is as follows (see BNO055")
    print("  datasheet for more information).")
    print("")
    print("  The instructions below can be used along with data from the bno055_raw script")
    print("  to calculate a set of calibration constants. Once done, enter those constants")
    print("  below to save them to the control board. Saved calibration constants will be")
    print("  applied each time the control board is powered on.")
    print("")
    print("  Accelerometer Offsets (x, y, z):")
    print("    Changes the \"zero\" location for each axis of the accelerometer. Adjust so")
    print("    accelerometer reads as close to zero as possible while stationary, except")
    print("    for axis with gravity, which should read +9.8 in the direction of gravity.")
    print("    Range: -500 LSB to +500 LSB; 1 m/s^2 = 100 LSB")
    print("  Accelerometer Radius:")
    print("    Distance between vehicle's point of rotation and the sensor.")
    print("    Range: -2048 LSB to +2048 LSB; 1 m = 100 LSB")
    print("  Gyroscope Offsets (x, y, z): ")
    print("    Changes the\"zero\" location for each axis of the gyroscope. Adjust so")
    print("    gyroscope reads as close to zero as possible while stationary.")
    print("    Range: -2000 LSB to 2000 LSB; 1 dps = 16 LSB")
    print("")
    print("  Enter Calibration constants")
    cal = cb.BNO055Calibration()
    cal.accel_offset_x = prompt_int("    Accel Offset X: ")
    cal.accel_offset_y = prompt_int("    Accel Offset Y: ")
    cal.accel_offset_z = prompt_int("    Accel Offset Z: ")
    cal.accel_radius =   prompt_int("    Accel Radius:   ")
    cal.gyro_offset_x =  prompt_int("    Gyro Offset X:  ")
    cal.gyro_offset_y =  prompt_int("    Gyro Offset Y:  ")
    cal.gyro_offset_z =  prompt_int("    Gyro Offset Z:  ")
    if prompt_yn("  Write above calibration to control board?", "y"):
        ack = cb.store_bno055_calibration(cal)
        if ack == cb.AckError.NONE:
            print("")
            print("  Calibration saved. It will be applied after control board power on.")
        else:
            print("")
            print("  Failed to save calibration!")
            return 1
    return 0


def run(cb: ControlBoard, s: Simulator) -> int:
    if platform.system() == "Windows":
        os.system("cls")
    else:
        os.system("clear")

    print("BNO055 IMU Calibration")
    print("================================================================================")

    # Simulation check
    if s is not None:
        print("  Sensor calibration cannot occur under simulation!")
        return 1
    
    # Check if calibration is already saved. If so, prompt for deletion
    res = check_existing_cal(cb, s)
    if res != 0:
        return res

    # Calibration type menu
    print("  Calibration Types")
    print("    1. Guided Calibration (pick this if unsure)")
    print("    2. Manual Calibration")
    choice = 0
    while choice > 2 or choice < 1:
        choice = prompt_int("  Select Calibration Type: ")
    if choice == 1:
        return guided_calibration(cb, s)
    elif choice == 2:
        return manual_calibration(cb, s)
    else:
        return 1