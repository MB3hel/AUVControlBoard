#!/usr/bin/env python3

from control_board import ControlBoard, Simulator
import time
import platform
import os


def run(cb: ControlBoard, s: Simulator) -> int:

    if s is not None:
        s.set_robot_rot(*s.euler_to_quat(2.6361095411338895, 2.3100118717539235, 49.73981961117051))

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
    if s is None:
        # Only valid if not in simulation
        print("Set axis config...", end="")
        if cb.set_bno055_axis(ControlBoard.BNO055Axis.P6) == ControlBoard.AckError.NONE:
            print("Done.")
        else:
            print("Fail.")
            return 1

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
        # print("Accum Pitch: {}".format(imu_data.accum_pitch))
        # print("Accum Roll: {}".format(imu_data.accum_roll))
        # print("Accum Yaw: {}".format(imu_data.accum_yaw))
        print("W: {}".format(imu_data.quat_w))
        print("X: {}".format(imu_data.quat_x))
        print("Y: {}".format(imu_data.quat_y))
        print("Z: {}".format(imu_data.quat_z))
        print()
        print("Depth: {}".format(depth_data.depth))

        time.sleep(0.1)