/*
 * Copyright 2023 Marcus Behel
 * 
 * This file is part of AUVControlBoard-Firmware.
 * 
 * AUVControlBoard-Firmware is free software: you can redistribute it and/or modify it under the terms of the GNU 
 * General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 * 
 * AUVControlBoard-Firmware is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even 
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for 
 * more details.
 * 
 * You should have received a copy of the GNU General Public License along with AUVControlBoard-Firmware. If not, see 
 * <https://www.gnu.org/licenses/>. 
 * 
 */

#pragma once


// Version of the firmware
// THESE ARE ALL SET WHEN RUNNING package.sh!!!
// Type is a character indicating release type (eg a = alpha, b = beta, c = release candidate, space = release)
// BUILD is a number indicating pre-release revision
// If type is a space (' ') build is ignored
#define FW_VER_MAJOR 1
#define FW_VER_MINOR 1
#define FW_VER_REVISION 0
#define FW_VER_TYPE 'b'
#define FW_VER_BUILD 1
