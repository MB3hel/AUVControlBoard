#!/usr/bin/env python3

import argparse
import sys
import os
from control_board import ControlBoard, Simulator
import importlib
from serial import SerialException


def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description="Test script launcher")
    parser.add_argument("-s", dest="sim", action="store_true", help="Run via simulator")
    parser.add_argument("-p", dest="port", type=str, default="", help="Serial port to use (ignored with -s). Defaults to /dev/ttyACM0.")
    parser.add_argument("-d", dest="debug", action="store_true", help="Enable debug messages")
    parser.add_argument("-q", dest="quiet", action="store_true", help="Suppress debug log from control board.")
    parser.add_argument("script", type=str, help="Name of script to run. Must be a .py script in same directory with a run() function")
    args = parser.parse_args()


    # Make sure script exists and load it
    if args.script.find("/") != -1:
        print("Invalid script.")
        return 1
    if args.script.endswith(".py"):
        args.script = args.script[:-3]
    this_dir = os.path.dirname(__file__)
    if not os.path.exists(os.path.join(this_dir, "{}.py".format(args.script))):
        print("Invalid script.")
        return 1
    
    mod = importlib.import_module(args.script)
    if not hasattr(mod, "run"):
        print("Script missing run function")


    # Create connection to the control board or simulator
    if args.sim:
        try:
            s = Simulator(args.debug, args.quiet)
            s.reset_vehicle()
            cb = s.control_board
            res = mod.run(cb, s)
            if isinstance(res, int):
                return res
            else:
                return 0
        except ConnectionRefusedError:
            print("Failed to connect to simulator.")
            return 1
    else:
        if args.port == "":
            args.port = "/dev/ttyACM0"
        try:
            cb = ControlBoard(args.port, args.debug, args.quiet)
            res = mod.run(cb, None)
            if isinstance(res, int):
                return res
            else:
                return 0
        except SerialException:
            print("Failed to open serial port.")
            return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
