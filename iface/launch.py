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
# Date: May 12, 2023
# Version: 1.0.3
################################################################################

import argparse
import sys
import os
from control_board import ControlBoard, Simulator
import importlib
from serial import SerialException

#####################################
# KEEP THESE AND IN THIS ORDER!!!
import vehicle
if os.path.exists(os.path.join(os.path.dirname(__file__), "user_vehicles.py")):
    import user_vehicles
#####################################

from vehicle import Vehicle, all_vehicles, default_vehicle
from typing import Dict, Tuple


# List of keys
vehicles = []


def configure_vehicle(cb: ControlBoard, vehicle: str, sim: bool) -> bool:
    global vehicles
    print("Applying Vehicle configuration...", end="")
    if vehicle not in vehicles:
        print("  Failed. Unknown vehicle: '{0}'.".format(vehicle))
        return False
    vehicle_tuple = all_vehicles[vehicle]
    if sim:
        vehicle_obj = vehicle_tuple[1]
    else:
        vehicle_obj = vehicle_tuple[0]
    ack, where = vehicle_obj.configure(cb)
    if ack != ControlBoard.AckError.NONE:
        print("Failed. '{0}' failed with AckError {1}.".format(where, ack))
        return False
    print("Done.")
    return True


def main():
    global vehicles
    vehicles = list(all_vehicles.keys())

    # Parse arguments
    parser = argparse.ArgumentParser(description="Interface script launcher")
    parser.add_argument("-s", dest="sim", action="store_true", help="Run via simulator")
    parser.add_argument("-p", dest="port", type=str, default="", help="Serial port to use (ignored with -s). Defaults to /dev/ttyACM0.")
    parser.add_argument("-d", dest="debug", action="store_true", help="Enable debug messages")
    parser.add_argument("-q", dest="quiet", action="store_true", help="Suppress debug log from control board.")
    parser.add_argument("-v", dest="vehicle", metavar="vehicle", type=str, default=default_vehicle, choices=vehicles, 
                        help="Choose a vehicle configuration to apply. Choices: {0}. Default: {1}.".format(vehicles, default_vehicle))
    parser.add_argument("script", type=str, help="Name of script to run. Must be a .py script in the same directory as launch.py or a subdirectory (relative paths only).")
    args = parser.parse_args()


    # Make sure script exists and load it
    if args.script.endswith(".py"):
        args.script = args.script[:-3]
    this_dir = os.path.abspath(os.path.dirname(__file__))
    if not os.path.exists(os.path.join(this_dir, "{}.py".format(args.script))):
        print("Invalid script.")
        return 1
    
    if args.script.startswith(".\\") or args.script.startswith("./"):
        args.script = args.script[2:]
    mod = importlib.import_module(args.script.replace("/", ".").replace("\\", "."))
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
            if not configure_vehicle(cb, args.vehicle, True):
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
            if not configure_vehicle(cb, args.vehicle, False):
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
