#!/usr/bin/env python3

from control_board import ControlBoard
import sys
import time
import threading
from serial import SerialException
from typing import Callable


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

    print("Set thruster inversions...", end="")
    if cb.set_tinv([True, True, False, False, True, False, False, True]) == ControlBoard.AckError.NONE:
        print("Done.")
    else:
        print("Fail.")
        return 1
    
    ############################################################################
    # Local mode set
    ############################################################################

    print("Setting local mode vector...", end="")
    #                x    y    z    p    r    y
    if cb.set_local(0.0, 0.3, 0.0, 0.0, 0.0, 0.0) == ControlBoard.AckError.NONE:
        print("Done.")
    else:
        print("Fail.")
        return 1
    
    t_stop = False
    def do_feed():
        while not t_stop:
            cb.feed_motor_watchdog()
            time.sleep(0.25)
    t = threading.Thread(daemon=True, target=do_feed)
    print("Press enter to stop...", end="")
    input("")
    t_stop = True
    t.join()

    print("Stopping...", end="")
    if cb.set_local(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) == ControlBoard.AckError.NONE:
        print("Done.")
    else:
        print("Fail.")
        return 1


    return 0

if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        exit(0)
    except SerialException:
        print("Serial communication failure!")
        exit(2)
