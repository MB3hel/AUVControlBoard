#!/usr/bin/env python3

from control_board import ControlBoard, Simulator
import time


def run(cb: ControlBoard, s: Simulator) -> int:
    ############################################################################
    # Setup
    ############################################################################
    print("Set motor matrix...", end="")
    mat = ControlBoard.MotorMatrix()
    #        MotorNum    x      y      z    pitch   roll     yaw
    mat.set_row(3,    [ -1,    -1,     0,     0,      0,     +1   ])
    mat.set_row(4,    [ +1,    -1,     0,     0,      0,     -1   ])
    mat.set_row(1,    [ -1,    +1,     0,     0,      0,     -1   ])
    mat.set_row(2,    [ +1,    +1,     0,     0,      0,     +1   ])
    mat.set_row(7,    [  0,     0,    -1,    -1,     -1,      0   ])
    mat.set_row(8,    [  0,     0,    -1,    -1,     +1,      0   ])
    mat.set_row(5,    [  0,     0,    -1,    +1,     -1,      0   ])
    mat.set_row(6,    [  0,     0,    -1,    +1,     +1,      0   ])
    err = cb.set_motor_matrix(mat)
    if err == ControlBoard.AckError.NONE:
        print("Done.")
    else:
        print("Fail.")
        print(err)
        return 1

    print("Set thruster inversions...", end="")
    if cb.set_tinv([True, True, False, False, True, False, False, True]) == ControlBoard.AckError.NONE:
        print("Done.")
    else:
        print("Fail.")
        return 1
    
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
