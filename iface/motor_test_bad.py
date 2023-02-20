#!/usr/bin/env python3

from control_board import ControlBoard, Simulator
import time
import threading


def feed_watchdog(cb: ControlBoard):
    while True:
        cb.feed_motor_watchdog()
        time.sleep(0.15)


def run(cb: ControlBoard, s: Simulator) -> int:
    if s is not None:
        print("motor_test does not work in simulation.")
        return 1

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
    if cb.set_motor_matrix(mat) == ControlBoard.AckError.NONE:
        print("Done.")
    else:
        print("Fail.")
        return 1
    
    wdog_thread = threading.Thread(target=feed_watchdog, daemon=True, args=(cb,))
    wdog_thread.start()

    ############################################################################
    # Settings
    ############################################################################
    speed = 0.5
    duration = 0.5

    ############################################################################
    # Motor test
    ############################################################################
    for idx in range(8):
        print("Running {}".format(idx+1))
        # Set desired speed
        speeds = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        speeds[idx] = speed
        attempt = 0

        # cb._set_tinv_no_ack([True, True, False, False, True, False, False, True])
        if cb.set_raw(speeds) != ControlBoard.AckError.NONE:
            print("Set raw failed!")
        
        start = time.time()
        while time.time() - start < duration:
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
    return 0
