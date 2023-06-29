################################################################################
# Copyright 2023 Marcus Behel
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
# Calibrate MS5837 depth sensor
################################################################################
# Author: Marcus Behel
# Date: June 29, 2023
# Version: 1.0.0
################################################################################

if __name__ == "__main__":
    print("Do not run this script directly. Use launch.py to run it.")
    exit(1)

from control_board import ControlBoard, Simulator
import time
import platform
import os


def prompt_float(msg: str) -> float:
    while True:
        num_str = input(msg)
        try:
            num = float(num_str)
            return num
        except ValueError:
            pass


def run(cb: ControlBoard, s: Simulator) -> int:
    ############################################################################
    # Query sensor status
    ############################################################################
    print("Query sensor status...", end="")
    res, bno055, ms5837 = cb.get_sensor_status()
    if res != ControlBoard.AckError.NONE:
        print("Fail.")
        return 1
    print("Done.")
    print("MS5837: {}".format("Ready" if ms5837 else "Not Ready"))
    print()

    time.sleep(1)

    print("Current calibration")
    ack, cal = cb.ms5837_read_calibration()
    if ack != cb.AckError.NONE:
        print("Control board communication failed.")
        return 1
    print("  Atmospheric Pressure (Pa): {}".format(cal.atm_pressure))
    print("  Fluid Density (kg/m^3): {}".format(cal.fluid_density))
    print("Press ctrl+c to exit without changing calibration.")
    input("Press enter to continue calibration...")
    print("")
    
    print("Atmospheric pressure will be determined by averaging 5 seconds of data.")
    input("Press enter to being sampling data...")
    print("Reading pressure data for 5 seconds...")
    ack = cb.read_ms5837_periodic(True)
    if ack != cb.AckError.NONE:
        print("Sensor communication failed.")
        return 1
    pressure_sum = 0
    pressure_samples = 0
    start_time = time.time()
    while time.time() - start_time < 5.0:
        pressure_sum += cb.get_ms5837_data().pressure
        pressure_samples += 1
        time.sleep(0.1)
    pressure_avg = pressure_sum / pressure_samples
    print("Average pressure: {} Pa".format(pressure_avg))
    print("Will be used as atmospheric pressure for calibration.")
    print("")
    
    print("Enter the density of water the vehicle is operating in")
    print("  Freshwater: ~997.0 kg/m^3")
    print("  Seawater:   ~1026.0 kg/m^3")
    fluid_density = prompt_float("Water density (kg / m^3): ")

    print("")
    cal.atm_pressure = pressure_avg
    cal.fluid_density = fluid_density
    print("Applying calibration...", end="")
    if cb.ms5837_write_calibration(cal) == cb.AckError.NONE:
        print("Done.")
    else:
        print("Fail.")
        return 1
    print("")

    print("New calibration")
    ack, cal = cb.ms5837_read_calibration()
    if ack != cb.AckError.NONE:
        print("Control board communication failed.")
        return 1
    print("  Atmospheric Pressure (Pa): {}".format(cal.atm_pressure))
    print("  Fluid Density (kg/m^3): {}".format(cal.fluid_density))

    return 0
