#!/usr/bin/env python3

from control_board import ControlBoard, Simulator


def run(cb: ControlBoard, s: Simulator) -> int:
    cb.reset()
    return 0

