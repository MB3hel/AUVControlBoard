#!/usr/bin/env python3

from control_board import ControlBoard
import sys
from serial import SerialException


def main() -> int:
    port = "/dev/ttyACM0"
    if len(sys.argv) > 1:
        port = sys.argv[1]
    cb = ControlBoard(port)
    cb.reset()
    return 0

if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        exit(0)
    except SerialException:
        print("Serial communication failure!")
        exit(2)
