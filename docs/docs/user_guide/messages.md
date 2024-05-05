# Messages

This section describes what specific messages are sent to / received from the control board and what they do / mean. This does not address how messages are constructed or sent. For such information, see [Communication Protocol](./comm_protocol.md).

*Note: The Communication Protocol page uses the term "payload" for the data being transferred and "message" for the formatted / fully constructed set of data. Here, what we refer to as "messages" are actually the "payload" data, not the constructed data.*

## Message Definition Conventions

In the following sections, the following standard is used to describe message contents:

- Each message's contents are shown as a comma separated list of bytes. Each comma separated item is a single byte, with one exception for parameters (as described below)
- Parameters are shown inside square brackets. Parameters represent a value that will be described in more detail below the message structure information. Parameters can be multiple bytes (even though they take only one entry between commas).
- ASCII characters are shown in single quotes. These are single byte unsigned ASCII characters.
- Numbers are not contained within any symbols. Numbers may be in hex (prefix 0x), binary (prefix 0b), or decimal (no prefix).


<hr />

## Types of Messages

- **Commands**: Messages instructing an action be taken. These messages must be acknowledged upon receipt. The acknowledgement typically contains no data. Sent from PC to control board.
- **Queries**: Messages requesting information. These messages must be acknowledged upon receipt. The acknowledgement will contain the requested information. Sent from PC to control board.
- **Acknowledgements**: A very specific type of message acknowledging receipt of another message (with an error code and optional data). Sent from control board to PC.
- **Status Messages**: Unprompted messages containing information about state / data changes. Sent from control board to PC.

## Commands and Queries

### Motor motion commands

**Raw Speed Set**  
Used to set motor speeds in `RAW` mode. This command has the following format.  
```none
'R', 'A', 'W', [speed_1], [speed_2], [speed_3], [speed_4], [speed_5], [speed_6], [speed_7], [speed_8]
```  
`[speed_n]`: The speed of thruster `n` from -1.0 to 1.0. A 32-bit float (little endian).  
This message will be acknowledged. The acknowledge message will contain no result data.

**Local Speed Set**  
Used to set motor speeds in `LOCAL` mode. This command has the following format  
```none
'L', 'O', 'C', 'A', 'L', [x], [y], [z], [xrot], [yrot], [zrot]
```  
`[x]`, `[y]`, `[z]`, `[xrot]`, `[yrot]`, `[zrot]`: Speed for each DoF relative to the robot -1.0 to 1.0. A 32-bit float (little endian).  
This message will be acknowledged. The acknowledge message will contain no result data.

**Global Speed Set**  
Used to set motor speeds in `GLOBAL` mode. This command has the following format  
```none
'G', 'L', 'O', 'B', 'A', 'L', [x], [y], [z], [pitch_spd], [roll_spd], [yaw_spd]
```  
`[x]`, `[y]`, `[z]`: Speed for each "world-relative" (pitch and roll compensated) DoF -1.0 to 1.0. 32-bit float (little endian).  
`[pitch_spd]`, `[roll_spd]`, `[yaw_spd]`: Rate of change of vehicle pitch, roll, and yaw -1.0 to 1.0.  
This message will be acknowledged. The acknowledge message will contain no result data. Note that if the IMU is not working properly, this command will be acknowledged with the "Invalid Command" error code. *This can occur if the axis config of the IMU is changed immediately before issuing this command.*

**Orientation Hold Speed Set (Variant 1)**  
Used to set motor speeds in `ORIENTATION_HOLD` mode using a speed for yaw. This command has the following format  
```none  
'O', 'H', 'O', 'L', 'D', '1', [x], [y], [z], [yaw_spd], [target_pitch], [target_roll]
```  
Each value is a 32-bit float little endian.

Pitch and roll are euler angles (in degrees). These are intrinsic euler angles (z-x'-y'' convention per the control board's coordinate system).

x, y, z, and yaw_spd are x, y, z, and yaw_spd just as in global mode.

This message will be acknowledged with no data. Note that if the IMU or depth sensor is not working properly, this command will be acknowledged with the "Invalid Command" error code. *This can occur if the axis config of the IMU is changed immediately before issuing this command.*

**Orientation Hold Speed Set (Variant 2)**  
Used to set motor speeds in `ORIENTATION_HOLD` mode using a PID to maintain a target yaw. This command has the following format  
```none  
'O', 'H', 'O', 'L', 'D', '2', [x], [y], [z], [target_pitch], [target_roll], [target_yaw]
```  
Each value is a 32-bit float little endian. 

Target Pitch, roll, and yaw are euler angles (in degrees). These are intrinsic euler angles (z-x'-y'' convention per the control board's coordinate system).

x, y, and z are speeds in the x, y, and z DoFs the same as in `GLOBAL` mode.

This message will be acknowledged with no data. Note that if the IMU or depth sensor is not working properly, this command will be acknowledged with the "Invalid Command" error code. *This can occur if the axis config of the IMU is changed immediately before issuing this command.*

**Stability Assist Speed Set (Variant 1)**  
Used to set motor speeds in `STABILITY_ASSIST` mode using a speed for yaw. This command has the following format  
```none  
'S', 'A', 'S', 'S', 'I', 'S', 'T', '1', [x], [y], [yaw_spd], [target_pitch], [target_roll], [target_depth]
```  
Each value is a 32-bit float little endian.

Pitch and roll are euler angles (in degrees). These are intrinsic euler angles (z-x'-y'' convention per the control board's coordinate system).

Depth is in meters where negative numbers are below the surface.

x, y, and yaw_spd are x, y, and yaw_spd just as in global mode.

This message will be acknowledged with no data. Note that if the IMU or depth sensor is not working properly, this command will be acknowledged with the "Invalid Command" error code. *This can occur if the axis config of the IMU is changed immediately before issuing this command.*

**Stability Assist Speed Set (Variant 2)**  
Used to set motor speeds in `STABILITY_ASSIST` mode using a PID to maintain a target yaw. This command has the following format  
```none  
'S', 'A', 'S', 'S', 'I', 'S', 'T', '2', [x], [y], [target_pitch], [target_roll], [target_yaw], [target_depth]
```  
Each value is a 32-bit float little endian. 

Target Pitch, roll, and yaw are euler angles (in degrees). These are intrinsic euler angles (z-x'-y'' convention per the control board's coordinate system).

Depth is in meters where negative numbers are below the surface.

x and y are speeds in the x and y DoFs the same as in `GLOBAL` mode.

This message will be acknowledged with no data. Note that if the IMU or depth sensor is not working properly, this command will be acknowledged with the "Invalid Command" error code. *This can occur if the axis config of the IMU is changed immediately before issuing this command.*

**Feed Motor Watchdog**  
Used to feed the motor watchdog so it does not kill the motors. This command has the following format  
```none
'W', 'D', 'G', 'F'
```
This message will be acknowledged. The acknowledge message will contain no result data.


### Vehicle Configuration Commands

**Motor Matrix Set**  
Motor matrix set command is used to set a single row of the motor matrix. It has the following format  
```none
'M', 'M', 'A', 'T', 'S', [thruster_num], [x], [y], [z], [pitch], [roll], [yaw]
```  
`[thruster_num]`: A single byte who'se unsigned value is the thruster number the row data should be set for (1-8).  
`[x]`, `[y]`, `[z]`, `[pitch]`, `[roll]`, `[yaw]`: Columns of the motor matrix row being set. Each is a 32-bit float (little endian).  
This message will be acknowledged. The acknowledge message will contain no result data.

**Motor Matrix Update**  
Motor matrix update command is used to inform the control board the motor matrix has changed. Causes the control board to perform some calculations with the new motor matrix. This should be sent after writing all rows of the motor matrix that should change using the motor matrix set command. The motor matrix update command has the following format  
```none
'M', 'M', 'A', 'T', 'U'
```
This message will be acknowledged. The acknowledge message will contain no result data.

**Thruster PWM Parameter Config**  
Configure PWM settings for thrusters.  
```none
'T', 'P', 'W', 'M', [pwm_period], [pwm_zero], [pwm_range]
```  
Each value is an unsigned 16-bit integer (little endian). `pwm_period` is the PWM signal period in microseconds (determines PWM frequency / update rate for ESCs). `pwm_zero` is the pulse width for zero speed in microseconds (typically 1500). `pwm_range` is the deviation from zero to achieve max speed (when added) or min speed (when subtracted) such that the pulse width `pwm_zero + pwm_range` microseconds is full forward speed and the pulse width `pwm_zero - pwm_range` microseconds is full reverse speed.  
This message will be acknowledged. The acknowledge message will contain no result data.

**Thruster Inversion Set**  
Thruster inversion set command is used to invert the positive and negative direction of thrusters. It has the following format  
```none
'T', 'I', 'N', 'V', [inv]
```  
`[inv]`: A single byte where each bit represents the inversion status of a thruster. The MSB (bit 7) corresponds to thruster 8 and the LSB corresponds to thruster 1 (bit + 1 = thruster). A bit value of 1 means the thruster is inverted. A bit value of 0 means the thruster is not inverted.  
This message will be acknowledged. The acknowledge message will contain no result data.

**Relative DoF Speed Set**  
Used to set relative speeds of motion in each DoF. There are two groups: linear (x, y, z) and angular (xrot, yrot, zrot). Within each group, use 1.0 for the fastest DoF. Other DoFs in the group are percentages of the fastest speed (from 0.0 to 1.0). This message has the following format  
```none
'R', 'E', 'L', 'D', 'O', 'F', [x], [y], [z], [xrot], [yrot], [zrot]
```  
`[x]`, `[y]`, `[z]`, `[xrot]`, `[yrot]`, `[zrot]`: 32-bit little endian floats.  
This message will be acknowledged. The acknowledge message will contain no result data.

**PID Tune Command**  
Used to tune PID controllers. The command has the following format  
```none  
'P', 'I', 'D', 'T', 'N', [which], [kp], [ki], [kd], [limit], [invert]
```  
`[which]` indicates which PID to tune ('X' = xrot, 'Y' = yrot, 'Z' = zrot, 'D' = depth hold).  
`[kp]`, `[ki]`, `[kd]` are proportional, integral, derivative, and feed-forward gains (32-bit float little endian).  
`[limit]` Is the PID controller's max output (limits max speed in the controlled DoF). Must be between 0.0 and 1.0. 32-bit float little endian.  
`[invert]` Set to one to invert PID output. Zero otherwise.


### Sensor Commands and Queries

**Sensor Status Query**  
Check which IMU and depth sensor is currently in use 
```none
'S', 'S', 'T', 'A', 'T'
```
This message will be acknowledged. If acknowledged with no error, the response will contain data in the following format  
```none
[which_imu], [which_depth]
```  
`[which_imu]`: Single byte indicating which IMU is in use  
- No IMU: 0  
- Simulated IMU (under simulator hijack): 1  
- BNO055 IMU: 2  

`[which_depth]`: Single byte indicating which depth sensor is in use  
- No Depth sensor: 0  
- Simulated depth sensor (under simulator hijack): 1  
- MS5837 Depth sensor: 2

**IMU Read**  
Reads IMU data once.  
```none
'I', 'M', 'U', 'R'
```  
This message will be acknowledged. If acknowledged with no error, the response will contain data in the following format. Note that this is the same format as the data contained within the BNO055 data status message.  
```none
[quat_w], [quat_x], [quat_y], [quat_z], [accum_pitch], [accum_roll], [accum_yaw]
```  
Each value is a 32-bit float, little endian. `quat_` values are components of the orientation quaternion. `accum_` values are accumulated euler angles.

**IMU Raw Read**  
Reads raw IMU data once.  
```none
'I', 'M', 'U', 'W'
```  
This message will be acknowledged. If acknowledged with no error, the response will contain data in the following format. Note that this is the same format as the data contained within the BNO055 data status message.  
```none
[accel_x], [accel_y], [accel_z], [gyro_x], [gyro_y], [gyro_z]
```  
Each value is a 32-bit float, little endian.


**IMU Periodic Read**  
Used to enable / disable periodic reading of IMU data. This will only impact data being sent from control board to the pc. The control board itself will continue to read and use IMU data.  
```none
'I', 'M', 'U', 'P', [enable]
```  
`[enable]` is an 8-bit integer with a value of either 1 or 0. If 1, reading data periodically is enabled. If 0, reading data periodically is disabled.  
This message will be acknowledged. The acknowledge message will contain no result data.

**Depth Read**  
Reads depth sensor data once.  
```none
'D', 'E', 'P', 'T', 'H', 'R'
```  
This message will be acknowledged. If acknowledged with no error, the response will contain data in the following format. Note that this is the same format as the data contained within the MS5837 data status message.  
```none
[depth_m], [pressure_pa], [temp_c]
```  
`depth_m` is a 32-bit float, little endian (meters below surface). `pressure_pa` is a 32-bit float, little endian (measured pressure in Pa). `temp_c` is a 32-bit float, little endian (temperature of air / water).

**Depth Periodic Read**  
Used to enable / disable periodic reading of MS5837 data. This will only impact data being sent from control board to the pc. The control board itself will continue to read and use depth sensor data.  
```none
'D', 'E', 'P', 'T', 'G', 'P', [enable]
```  
`[enable]` is an 8-bit integer with a value of either 1 or 0. If 1, reading data periodically is enabled. If 0, reading data periodically is disabled.  
This message will be acknowledged. The acknowledge message will contain no result data.


### BNO055 IMU Configuration

**BNO055 IMU Axis Configure Command**  
Used to configure the BNO055 IMU's axis orientation. *Note: This will also reset the accumulated euler angles to zero*.  
```none
'B', 'N', 'O', '0', '5', '5', 'A', [config]
```
`[config]`: A single byte. The value of this byte is between 0 and 7 (inclusive) representing on of the BNO055 axis configs (P0 to P7) as described in the BNO055 datasheet. *Note: Changing the axis config changes IMU mode. Thus, there will be a brief time afterwards where the IMU may report zeros for all data.*  
This message will be acknowledged. The acknowledge message will contain no result data. Note that if the BNO055 is not the active IMU (see sensor status query), this will be acknowledged using the INVALID_CMD error code.

**Save BNO055 Stored Calibration Command**  
This command is used to store a set of calibration constants for the BNO055 to the control board. This will write the "stored calibration constants". This command will also cause the IMU to be reconfigured (this can take some time, so acknowledgements for this command may take longer than most). The command has the following format  
```none
'S', 'C', 'B', 'N', 'O', '0', '5', '5', 'S', [accel_offset_x], [accel_offset_y], [accel_offset_z], [accel_radius], [gyro_offset_x], [gyro_offset_y], [gyro_offset_z]
```  
Each value is a signed 16-bit integer. The meaning of each value is described in the BNO055 datasheet.  
This message will be acknowledged. The acknowledge message will contain no result data. Note that if the BNO055 is not the active IMU (see sensor status query), this will be acknowledged using the INVALID_CMD error code.

**Erase BNO055 Stored Calibration Command**  
This command is used to erase calibration constants for the BNO055 from the control board. This will erase the "stored calibration constants". This command will also cause the IMU to be reconfigured (this can take some time, so acknowledgements for this command may take longer than most). The command has the following format  
```none
'S', 'C', 'B', 'N', 'O', '0', '5', '5', 'E'
```  
This message will be acknowledged. The acknowledge message will contain no result data. Note that if the BNO055 is not the active IMU (see sensor status query), this will be acknowledged using the INVALID_CMD error code.

**Reset BNO055 Command**  
This command is used to reset / reconfigure the BNO055. This is typically used to clear any auto generated calibration constants. The command has the following format  
```none
'B', 'N', 'O', '0', '5', '5', 'R', 'S', 'T'
```  
This message will be acknowledged. The acknowledge message will contain no result data. Note that if the BNO055 is not the active IMU (see sensor status query), this will be acknowledged using the INVALID_CMD error code.

**Read BNO055 Stored Calibration Query**  
This command is used to read a set of calibration constants for the BNO055 from the control board. This will read the "stored calibration constants". The command has the following format  
```none
'S', 'C', 'B', 'N', 'O', '0', '5', '5', 'R'
```  
This message will be acknowledged. If acknowledged with no error, the response will contain data in the following format.  
```none
[valid], [accel_offset_x], [accel_offset_y], [accel_offset_z], [accel_radius], [gyro_offset_x], [gyro_offset_y], [gyro_offset_z]
```  
`valid` is an 8-bit integer. A value of 0 indicates that no calibration is stored on the control board (other values have no meaning). A value of 1 indicates that a calibration is stored (other values are that calibration).  
All other values in the acknowledge data are signed 16-bit integers. The meaning of these integers is described in the BNO055 datasheet.

**Read BNO055 Live Calibration Status Query**  
This command is used to read the status of the BNO055's calibration routine. Note that this reads the value directly from the BNO055. This value is meaningless if a calibration was manually applied to the sensor. Thus, this is only useful if any "stored calibration constants" are first erased. The command has the following format  
```none
'B', 'N', 'O', '0', '5', '5', 'C', 'S'
```  
This message will be acknowledged. Note that if the IMU is not working properly, this command will be acknowledged with the "Invalid Command" error code. If acknowledged with no error, the response will contain data in the following format.  
```none
[status]
```  
`status` is an 8-bit integer. The value of `status` is the value of the BNO055's `CALIB_STAT` register. The meaning of this number is described in the BNO055 datasheet. Note that if the BNO055 is not the active IMU (see sensor status query), this will be acknowledged using the INVALID_CMD error code.


**Read BNO055 Live Calibration Values Query**  
This command is used to read a set of calibration constants from the BNO055. This will read the "live calibration constants" directly from the BNO055. Note that the calibration constants are only valid if the calibration status from the BNO055 is 3 for the accelerometer and gyroscope. The command has the following format  
```none
'B', 'N', 'O', '0', '5', '5', 'C', 'V'
```  
This message will be acknowledged. Note that if the IMU is not working properly, this command will be acknowledged with the "Invalid Command" error code. If acknowledged with no error, the response will contain data in the following format.  
```none
[accel_offset_x], [accel_offset_y], [accel_offset_z], [accel_radius], [gyro_offset_x], [gyro_offset_y], [gyro_offset_z]
```  
All  values in the acknowledge data are signed 16-bit integers. The meaning of these integers is described in the BNO055 datasheet. Note that if the BNO055 is not the active IMU (see sensor status query), this will be acknowledged using the INVALID_CMD error code.


### MS5837 Depth Sensor Configuration

**Read MS5837 Calibration Query**  
This command is used to read the values of the MS5837 calibration constants. The command has the following format  
```none
'M', 'S', '5', '8', '3', '7', 'C', 'A', 'L', 'G'
```  
This message will be acknowledged. If acknowledged with no error, the response will contain data in the following format.  
```none
[atm_pressure], [fluid_density]
```  
Both values are 32-bit little endian floats.

**Write MS5837 Calibration Command**  
This command is used to write the values of the MS5837 calibration constants. The command has the following format  
```none
'M', 'S', '5', '8', '3', '7', 'C', 'A', 'L', 'S', [atm_pressure], [fluid_density]
```  
Both values are 32-bit little endian floats.  
This message will be acknowledged. The acknowledge message will contain no result data.

### Misc Commands and Queries

**Reset Command**  
This command is used to rest the control board itself. This will reset the microcontroller on the control board, thus the USB device will disconnect and reconnect (note that if your program still holds the port when this happens, the USB device will likely be assigned a different port number).
```none
'R', 'E', 'S', 'E', 'T', 0x0D, 0x1E
```
This message is **not** acknowledged.

**Last Reset Cause Query**  
Get error code for last system reset cause of the control board. Generally not useful for end users, except for reporting errors. Mainly a debug / development tool. See error codes in firmware source `debug.h`.
```none
'R', 'S', 'T', 'W', 'H', 'Y'
```  
This message will be acknowledged. If acknowledged with no error, the response will contain data in the following format.
```none
[error_code]
```  
`error_code` is a 32-bit integer (signed), little endian.

**Simulator Hijack Command**  
This command is used by the simulator to hijack a real control board. This allows the simulator to pass certain information to and receive certain information from the control board. This enables testing of the actual firmware and reproducing bugs under simulation. The command has the following format.  
```none
'S', 'I', 'M', 'H', 'I', 'J', 'A', 'C', 'K', [hijack]
```  
`[hijack]` is an 8-bit integer (unsigned) with a value of 1 or 0. If 1, the control board is put into simulator hijack mode. If 0, it is removed from simulator hijack mode.  
This message will be acknowledged. The acknowledge message will contain no result data.

**Simulator Data Command**  
This command is used by the simulator to send simulated sensor data to a hijacked control board.  
```none
'S', 'I', 'M', 'D', 'A', 'T', [w], [x], [y], [z], [depth]
```  
All values are little endian floats (32-bit). `x`, `y`, `z`, `w` are current quaternion (IMU data) `depth` is current depth (depth sensor data).


**Version Info Query**  
Get the version info from the control board.  
```none
'C', 'B', 'V', 'E', 'R'
```  
This message will be acknowledged. If acknowledged with no error, the response will contain data in the following format  
```none
[cb_ver],[fw_ver_major],[fw_ver_minor],[fw_ver_revision],[fw_ver_type],[fw_ver_build]
```  
Each value is an unsigned 8-bit integer. All are simply interpreted as numbers, except for `fw_ver_type`, which is an ASCII character.  
`cb_ver`: Version of the control board hardware (CBv1 or CBv2 = 1 or 2; 0 = SimCB in simulator)  
`fw_ver_major`: Major version of firmware running on the control board  
`fw_ver_minor`: Minor version of firmware running on the control board  
`fw_ver_revision`: Revision version of firmware running on the control board  
`fw_ver_type`: Type of firmware release. 'a' = alpha, 'b' = beta, 'c' = release candidate (rc), ' ' (space) = full release  
`fw_ver_build`: Build number for pre-release firmware. Should be ignored for fw_ver_type release (' ')


## Acknowledgements

An acknowledgement message has the following format  
```none
'A', 'C', 'K', [ack_id], [error_code], [result]
```  
`[ack_id]`: The ID of the message being acknowledged. Unsigned 16-bit integer (big endian).  
`[error_code]`: A single byte error code.  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;0 = None: No error.  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;1 = Unknown Message: Control board does not recognize the message.  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;2 = Invalid arguments: Message is recognized, but arguments are invalid  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;3 = Invalid Command: Command is known, but is not valid at this time.  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;255 = Reserved: Control board will not use this code. Typically used as timeout.  
`[result]`: Optional data of variable size attached to the acknowledge message. Its size, format, and meaning depends on the message being acknowledged.


## Status Messages

**Motor Watchdog Status**  
Motor watch status message is used by the control board to notify the PC about changes to motor (watchdog) state. It has the following format  
```none
'W', 'D', 'G', 'S', [status]
```
`[status]` is a single byte. A value of 1 indicates the motors are enabled. A value of zero indicates the motors are currently killed by the watchdog.

**IMU Data Status**  
Used by the control board to periodically send IMU data to the PC. Only sent when IMU periodic reads are enabled via the IMU periodic read command. The message has the following format  
```none
'I', 'M', 'U', 'D', [quat_w], [quat_x], [quat_y], [quat_z], [accum_pitch], [accum_roll], [accum_yaw]
```  
Each value is a 32-bit float, little endian. `quat_` values are components of the orientation quaternion. `accum_` values are accumulated euler angles.

**Depth Data Status**  
Used by the control board to periodically send IMU data to the PC. Only sent when BNO055 periodic reads are enabled via the BNO055 periodic read command. The message has the following format  
```none
'D', 'E', 'P', 'T', 'H', 'D', [depth_m], [pressure_pa], [temp_c]
```  
`depth_m` is a 32-bit float, little endian (meters below surface). `pressure_pa` is a 32-bit float, little endian (measured pressure in Pa). `temp_c` is a 32-bit float, little endian (temperature of air / water).


**Debug Status Messages**  
Used only during development. These will not occur on release builds of the firmware. These are arbitrary messages sent by the control board to the PC for the firmware developer's use during development. They have the following format
```none
'D', 'E', 'B', 'U', 'G', [msg]
```  
`msg` is an ascii string.


**Debug Data Status Messages**  
Used only during development. These will not occur on release builds of the firmware. These are arbitrary messages sent by the control board to the PC for the firmware developer's use during development. They have the following format
```none
'D', 'B', 'G', 'D', 'A', 'T', [msg]
```  
`msg` is arbitrary data.

**Heartbeat Status Messages**  
Sent from control board periodically to indicate that it still exists and is operating as expected. This is generally ignored by end users. It is mostly intended to ensure communication occurs periodically in SimCB so connection drops are detectable.  
```none
'H', 'E', 'A', 'R', 'T', 'B', 'E', 'A', 'T'
```

**Simulator Status Message**  
Sent from a simulator hijacked control board periodically to provide simulator with state and motor speed information.  
```none
'S', 'I', 'M', 'S', 'T', 'A', 'T', [t1], [t2], [t3], [t4], [t5], [t6], [t7], [t8], [mode], [wdog_killed]
```  
Each value `t1` to `t8` is a 32-bit little endian float representing thruster speeds 1 - 8 respectively.  
`mode` is an unsigned 8-bit integer indicating the control board's current operating mode from one of the following

- Raw = 0
- Local = 1
- Global = 2
- Sassist = 3
- Ohold = 5

`wdog_killed` is an unsigned 8-bit integer indicating if the control board's motors are killed due to motor watchdog timeout. 1 indicates that motors are killed. 0 indicates not killed.
