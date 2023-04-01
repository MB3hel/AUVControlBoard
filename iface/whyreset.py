#!/usr/bin/env python3

from control_board import ControlBoard, Simulator


def run(cb: ControlBoard, s: Simulator) -> int:
    if s is not None:
        print("Reset not supported in simulation.")
        return 1
    res, ec = cb.why_reset()
    if(res == cb.AckError.NONE):
        print("Reset cause: {}".format(ec))
    else:
        print("Query reset cause failed!")
    return 0

