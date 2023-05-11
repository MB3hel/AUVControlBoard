#!/usr/bin/env python3
################################################################################
# Copyright 2022-2023 Marcus Behel
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
# Control board script launcher. Handles connection to a control board or to
# the simulator and invokes the test script with instantiated objects.
################################################################################
# Author: Marcus Behel
# Date: May 10, 2023
# Version: 1.0.0
################################################################################

import argparse
import sys
import os
from control_board import ControlBoard, Simulator
import importlib
from serial import SerialException
from vehicle import SW8, Vehicle
from typing import Dict


vehicles: Dict[str, Vehicle] = {
    "sw8": SW8()
}
default_vehicle = "sw8"


def configure_vehicle(cb: ControlBoard, vehicle: str) -> bool:
    global vehicles
    print("Applying Vehicle configuration...", end="")
    if vehicle not in vehicles:
        print("  Failed. Unknown vehicle: '{0}'.".format(vehicle))
        return False
    vehicle_obj = vehicles[vehicle]
    ack, where = vehicle_obj.configure(cb)
    if ack != ControlBoard.AckError.NONE:
        print("Failed. '{0}' failed with AckError {1}.".format(where, ack))
        return False
    print("Done.")
    return True


def main():
    global vehicles, default_vehicle
    # Parse arguments
    parser = argparse.ArgumentParser(description="Interface script launcher")
    parser.add_argument("-s", dest="sim", action="store_true", help="Run via simulator")
    parser.add_argument("-p", dest="port", type=str, default="", help="Serial port to use (ignored with -s). Defaults to /dev/ttyACM0.")
    parser.add_argument("-d", dest="debug", action="store_true", help="Enable debug messages")
    parser.add_argument("-q", dest="quiet", action="store_true", help="Suppress debug log from control board.")
    parser.add_argument("-v", dest="vehicle", metavar="vehicle", type=str, default=default_vehicle, choices=list(vehicles.keys()), help="Choose a vehicle configuration to apply. Defaults to 'sw8'.")
    parser.add_argument("script", type=str, help="Name of script to run. Must be a .py script in same directory with a run() function")
    args = parser.parse_args()


    # Make sure script exists and load it
    if args.script.find("/") != -1:
        print("Invalid script.")
        return 1
    if args.script.endswith(".py"):
        args.script = args.script[:-3]
    this_dir = os.path.dirname(__file__)
    if not os.path.exists(os.path.join(this_dir, "scripts", "{}.py".format(args.script))):
        print("Invalid script.")
        return 1
    
    mod = importlib.import_module("scripts.{0}".format(args.script))
    if not hasattr(mod, "run"):
        print("Script missing run function")
        return 1


    # Create connection to the control board or simulator
    if args.sim:
        try:
            print("Connecting to simulator...", end="")
            s = Simulator(args.debug, args.quiet)
            print("Done.")
            cb = s.control_board
            if not configure_vehicle(cb, args.vehicle):
                return 1
            res = mod.run(cb, s)
            if isinstance(res, int):
                return res
            else:
                return 0
        except (ConnectionRefusedError, ConnectionResetError):
            print("Failed.")
            return 1
    else:
        if args.port == "":
            args.port = "/dev/ttyACM0"
        try:
            print("Connecting to {0}...".format(args.port), end="")
            cb = ControlBoard(args.port, args.debug, args.quiet)
            print("Done.")
            if not configure_vehicle(cb, args.vehicle):
                return 1
            res = mod.run(cb, None)
            if isinstance(res, int):
                return res
            else:
                return 0
        except SerialException:
            print("Failed.")
            return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
