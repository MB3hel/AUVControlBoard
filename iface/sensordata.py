#!/usr/bin/env python3

from control_board import ControlBoard
import sys
import time
from serial import SerialException
from typing import Callable
import platform
import os


def main() -> int:
    ############################################################################
    # Open communication with control board
    ############################################################################
    port = "/dev/ttyACM0"
    if len(sys.argv) > 1:
        port = sys.argv[1]
    cb = ControlBoard(port)

    ############################################################################
    # Setup
    ############################################################################
    print("Set axis config...", end="")
    if cb.set_bno055_axis(ControlBoard.BNO055Axis.P5) == ControlBoard.AckError.NONE:
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
        
        print("Grav X: {}".format(imu_data.grav_x))
        print("Grav Y: {}".format(imu_data.grav_y))
        print("Grav Z: {}".format(imu_data.grav_z))
        print()
        print("Pitch: {}".format(imu_data.euler_pitch))
        print("Roll: {}".format(imu_data.euler_roll))
        print("Yaw: {}".format(imu_data.euler_yaw))
        print()
        print("Accum Pitch: {}".format(imu_data.accum_pitch))
        print("Accum Roll: {}".format(imu_data.accum_roll))
        print("Accum Yaw: {}".format(imu_data.accum_yaw))
        print()
        print("Depth: {}".format(depth_data.depth))

        time.sleep(0.1)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        exit(0)
    except SerialException:
        print("Serial communication failure!")
        exit(2)
