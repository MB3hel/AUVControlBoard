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

<!--TODO: Future sensor config commands-->

### Motor Control Commands

**Raw Speed Set**  
Used to set motor speeds in `RAW` mode. This command has the following format.  
```none
'R', 'A', 'W', [speed_1], [speed_2], [speed_3], [speed_4], [speed_5], [speed_6], [speed_7], [speed_8]
```  
`[speed_n]`: The speed of thruster `n` from -1.0 to 1.0. A 32-bit float (little endian).  
This message will be acknowledged. The acknowledge message will contain no result data.

**Local Speed Set**  
Used to set motor speeds in `LOCAL` mode. This command has the following format.  
```none
'L', 'O', 'C', 'A', 'L', [x], [y], [z], [pitch], [roll], [yaw]
```  
`[x]`, `[y]`, `[z]`, `[pitch]`, `[roll]`, `[yaw]`: Speed for each DoF relative to the robot -1.0 to 1.0. A 32-bit float (little endian).  
This message will be acknowledged. The acknowledge message will contain no result data.

<!--TODO: Future other mode commands-->


### Other Commands

**Feed Motor Watchdog**  
Used to feed the motor watchdog so it does not kill the motors. This command has the following format  
```none
'W', 'D', 'G', 'F'
```
This message will be acknowledged. The acknowledge message will contain no result data.


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

<!--TODO: Future status messages (sensors)-->
