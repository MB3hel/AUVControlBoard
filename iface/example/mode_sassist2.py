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
# Move the vehicle using STABILITY_ASSIST mode variant 2
################################################################################
# Author: Marcus Behel
# Date: May 10, 2023
# Version: 1.0.0
################################################################################

if __name__ == "__main__":
    print("Do not run this script directly. Use launch.py to run it.")
    exit(1)

from control_board import ControlBoard, Simulator
import time
import threading


def run(cb: ControlBoard, s: Simulator) -> int:
    print("Setting sassist2 mode target...", end="")

    #                   x    y    p    r     h      d
    if cb.set_sassist2(0.0, 0.0, 5.0, 15.0, 175.0, -0.4) == ControlBoard.AckError.NONE:
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
    t.start()
    print("Press enter to stop...", end="")
    input("")
    t_stop = True
    t.join()

    print("Stopping...", end="")
    if cb.set_global(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) == ControlBoard.AckError.NONE:
        print("Done.")
    else:
        print("Fail.")
        return 1


    return 0

