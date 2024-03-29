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
# Get control board version information
################################################################################
# Author: Marcus Behel
# Date: September 24, 2023
# Version: 1.0.0
################################################################################

if __name__ == "__main__":
    print("Do not run this script directly. Use launch.py to run it.")
    exit(1)


from control_board import ControlBoard, Simulator


def run(cb: ControlBoard, s: Simulator) -> int:
    res, cb_ver, fw_ver = cb.get_version_info()
    if(res == cb.AckError.NONE):
        print("Hardware Version: {}".format(cb_ver))
        print("Firmware Version: {}".format(fw_ver))
    else:
        print("Query version info failed!")
    return 0


