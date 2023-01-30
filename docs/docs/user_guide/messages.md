# Messages

This section describes what specific messages are sent to / received from the control board and what they do / mean. This does not address how messages are constructed or sent. For such information, see [Communication Protocol](./comm_protocol.md).

*Note: The Communication Protocol page uses the term "payload" for the data being transferred and "message" for the formatted / fully constructed set of data. Here, what we refer to as "messages" are actually the "payload" data, not the constructed data.*

## Types of Messages

- **Commands**: Messages instructing an action be taken. These messages must be acknowledged upon receipt. The acknowledgement typically contains no data. Sent from PC to control board.
- **Queries**: Messages requesting information. These messages must be acknowledged upon receipt. The acknowledgement will contain the requested information. Sent from PC to control board.
- **Acknowledgements**: A very specific type of message acknowledging receipt of another message (with an error code and optional data). Sent from control board to PC.
- **Status Messages**: Unprompted messages containing information about state / data changes. Sent from control board to PC.


## Message Definition Conventions

In the following sections, the following standard is used to describe message contents:

- Each message's contents are shown as a comma separated list of bytes. Each comma separated item is a single byte, with one exception for parameters (as described below)
- Parameters are shown inside square brackets. Parameters represent a value that will be described in more detail below the message structure information. Parameters can be multiple bytes (even though they take only one entry between commas).
- ASCII characters are shown in single quotes. These are single byte unsigned ASCII characters.
- Numbers are not contained within any symbols. Numbers may be in hex (prefix 0x), binary (prefix 0b), or decimal (no prefix).


<hr />

## Commands

### Configuration Commands

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

**Thruster Inversion Set**  
Thruster inversion set command is used to invert the positive and negative direction of thrusters. It has the following format  
```none
'T', 'I', 'N', 'V', [inv]
```  
`[inv]`: A single byte where each bit represents the inversion status of a thruster. The MSB (bit 7) corresponds to thruster 8 and the LSB corresponds to thruster 1 (bit + 1 = thruster). A bit value of 1 means the thruster is inverted. A bit value of 0 means the thruster is not inverted.  
This message will be acknowledged. The acknowledge message will contain no result data.

**BNO055 IMU Axis Configure Command**  
Used to configure the BNO055 IMU's axis orientation. *Note: This will also reset the accumulated euler angles to zero*.  
```none
'B', 'N', 'O', '0', '5', '5', 'A', [config]
```
`[config]`: A single byte. The value of this byte is between 0 and 7 (inclusive) representing on of the BNO055 axis configs (P0 to P7) as described in the BNO055 datasheet. *Note: Changing the axis config changes IMU mode. Thus, there will be a brief time afterwards where the IMU may report zeros for all data.*  
This message will be acknowledged. The acknowledge message will contain no result data.

**Stability Assist Mode PID Tune Command**  
Used to tune stability assist mode PID controllers. It has the following format  
```none  
'S', 'A', 'S', 'S', 'I', 'S', 'T', 'T', 'N', [which], [kp], [ki], [kd], [kf], [limit]
```  
`[which]` indicates which PID to tune ('P' = pitch hold, 'R' = roll hold, 'Y' = yaw hold, 'D' = depth hold).  
`[kp]`, `[ki]`, `[kd]`, and `[kf]` are proportional, integral, derivative, and feed-forward gains (32-bit float little endian).  
`[limit]` Is the PID controller's max output (limits max speed in the controlled DoF). Must be between 0.0 and 1.0. 32-bit float little endian.


### Motor Control Commands

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
'L', 'O', 'C', 'A', 'L', [x], [y], [z], [pitch], [roll], [yaw]
```  
`[x]`, `[y]`, `[z]`, `[pitch]`, `[roll]`, `[yaw]`: Speed for each DoF relative to the robot -1.0 to 1.0. A 32-bit float (little endian).  
This message will be acknowledged. The acknowledge message will contain no result data.

**Global Speed Set**  
Used to set motor speeds in `GLOBAL` mode. This command has the following format  
```none
'G', 'L', 'O', 'B', 'A', 'L', [x], [y], [z], [pitch], [roll], [yaw]
```  
`[x]`, `[y]`, `[z]`, `[pitch]`, `[roll]`, `[yaw]`: Speed for each DoF relative to the world (pitch and roll compensated; not yaw compensated) -1.0 to 1.0. A 32-bit float (little endian).  
This message will be acknowledged. The acknowledge message will contain no result data. Note that if the IMU is not working properly, this command will be acknowledged with the "Invalid Command" error code. *This can occur if the axis config of the IMU is changed immediately before issuing this command.*

**Stability Assist Speed Set (Variant 1)**  
Used to set motor speeds in `STABILITY_ASSIST` mode using a speed for yaw. This command has the following format  
```none  
'S', 'A', 'S', 'S', 'I', 'S', 'T', 'S', 'T', '1', [x], [y], [yaw], [target_pitch], [target_roll], [target_depth]
```  
Each value is a 32-bit float little endian.  This message will be acknowledged with no data. Note that if the IMU or depth sensor is not working properly, this command will be acknowledged with the "Invalid Command" error code. *This can occur if the axis config of the IMU is changed immediately before issuing this command.*

**Stability Assist Speed Set (Variant 2)**  
Used to set motor speeds in `STABILITY_ASSIST` mode using a PID to maintain a target yaw. This command has the following format  
```none  
'S', 'A', 'S', 'S', 'I', 'S', 'T', 'S', 'T', '2', [x], [y], [target_pitch], [target_roll], [target_yaw], [target_depth]
```  
Each value is a 32-bit float little endian.  This message will be acknowledged with no data. Note that if the IMU or depth sensor is not working properly, this command will be acknowledged with the "Invalid Command" error code. *This can occur if the axis config of the IMU is changed immediately before issuing this command.*


### Other Commands

**Feed Motor Watchdog**  
Used to feed the motor watchdog so it does not kill the motors. This command has the following format  
```none
'W', 'D', 'G', 'F'
```
This message will be acknowledged. The acknowledge message will contain no result data.

**BNO055 Periodic Read**  
Used to enable / disable periodic reading of BNO055 data. This will only impact data being sent from control board to the pc. The control board itself will continue to read and use IMU data.  
```none
'B', 'N', 'O', '0', '5', '5', 'P', [enable]
```  
`[enable]` is an 8-bit integer with a value of either 1 or 0. If 1, reading data periodically is enabled. If 0, reading data periodically is disabled.  
This message will be acknowledged. The acknowledge message will contain no result data.

**MS5837 Periodic Read**  
Used to enable / disable periodic reading of MS5837 data. This will only impact data being sent from control board to the pc. The control board itself will continue to read and use depth sensor data.  
```none
'M', 'S', '5', '8', '3', '7', 'P', [enable]
```  
`[enable]` is an 8-bit integer with a value of either 1 or 0. If 1, reading data periodically is enabled. If 0, reading data periodically is disabled.  
This message will be acknowledged. The acknowledge message will contain no result data.

**Reset Command**  
This command is used to rest the control board itself. This will reset the microcontroller on the control board, thus the USB device will disconnect and reconnect (note that if your program still holds the port when this happens, the USB device will likely be assigned a different port number).
```none
'R', 'E', 'S', 'E', 'T', 0x0D, 0x1E
```


<hr />



## Queries

**Sensor Status Query**  
Gets the status of all sensors (BNO055 and MS5837).  
```none
'S', 'S', 'T', 'A', 'T'
```
This message will be acknowledged. If acknowledged with no error, the response will contain data in the following format  
```none
[status_byte]
```  
`[status_byte]`: Single byte containing bits for the status of all sensors. Bit 0 (LSB) is BNO055 status. Bit 1 is MS5837 status. A status bit of 1 indicates the sensor is "ready" (connected and can be used). A status bit of 0 indicates the sensor is "not ready".


**BNO055 Read**  
Reads BNO055 IMU data once.  
```none
'B', 'N', 'O', '0', '5', '5', 'R'
```  
This message will be acknowledged. If acknowledged with no error, the response will contain data in the following format. Note that this is the same format as the data contained within the BNO055 data status message.  
```none
[quat_w], [quat_x], [quat_y], [quat_z], [accum_pitch], [accum_roll], [accum_yaw]
```  
Each value is a 32-bit float, little endian. `quat_` values are components of the orientation quaternion. `accum_` values are accumulated euler angles.


**MS5837 Read**  
Reads MS5837 data once.  
```none
'M', 'S', '5', '8', '3', '7', 'R'
```  
This message will be acknowledged. If acknowledged with no error, the response will contain data in the following format. Note that this is the same format as the data contained within the MS5837 data status message.  
```none
[depth_m]
```  
`depth_m` is a 32-bit float, little endian.

<!--TODO: Future sensor queries-->

<hr />


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


<hr />

## Status Messages

**Motor Watchdog Status**  
Motor watch status message is used by the control board to notify the PC about changes to motor (watchdog) state. It has the following format  
```none
'W', 'D', 'G', 'S', [status]
```
`[status]` is a single byte. A value of 1 indicates the motors are enabled. A value of zero indicates the motors are currently killed by the watchdog.

**BNO055 Data Status**  
Used by the control board to periodically send IMU data to the PC. Only sent when BNO055 periodic reads are enabled via the BNO055 periodic read command. The message has the following format  
```none
'B', 'N', 'O', '0', '5', '5', 'D', [quat_w], [quat_x], [quat_y], [quat_z], [accum_pitch], [accum_roll], [accum_yaw]
```  
Each value is a 32-bit float, little endian. `quat_` values are components of the orientation quaternion. `accum_` values are accumulated euler angles.

**MS5837 Data Status**  
Used by the control board to periodically send IMU data to the PC. Only sent when BNO055 periodic reads are enabled via the BNO055 periodic read command. The message has the following format  
```none
'M', 'S', '5', '8', '3', '7', 'D', [depth_m]
```  
`depth_m` is a 32-bit float, little endian.


**Debug Status Messages**  
Used only during development. These will not occur on release builds of the firmware. These are arbitrary messages sent by the control board to the PC for the firmware developer's use during development.  They have the following format
```none
'D', 'E', 'B', 'U', 'G', [msg]
```  
`msg` is arbitrary data (usually ascii string).
