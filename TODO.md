# Firmware

## Control Modes
- Remove mode set command. Mode should just change when a speed set command for that mode is issued. Still allow getting the mode though.
- GLOBAL and STABILITY_ASSIST modes should not work if required sensors are not connected
- Instead of periodically recalculating speeds in global and stability assist mode, use a timer. This way, if setpoint is changing rapidly from computer recalculate will be skipped.
- Implement PID controller
- Implement depth hold PID, pitch hold PID, and roll hold PID for stability assist mode.
- Implement yaw hold PID for stability assist mode.

## Communication
- Implement reset command
- Implement sensor restart command. This command should reset sensor drivers to allow newly connected sensors to be detected without a hard reset which would drop USB communication.
- Split into three types of messages. Do not impact encoding of messages, just makes documentation easier and defines what needs an ACK.
    - Set = computer telling contorl board to do something
    - Get = computer requesting information from control board
    - Status = Information sent from control board to computer unprompted
- All "set" messages respond with an ACK message (somehow indicating what the acknowledge is for). Should also include a status code.

## Sensors
- IMU (BNO055)
    - Unit select register should be configured by state machine
    - Configure axes via messages from PC
    - Command to read sensor data once
    - Command to start / stop reading sensor data periodically
    - Implement calibration status register read
    - Command to read calibration status from IMU
    - If too many i2c errors in a row, assume the sensor is no longer connected and reset state machine
- Depth Sensor (MS5837)
    - Configure fluid density via messages from PC
    - Calibrate via messages from PC
    - Command to read sensor data once
    - Command to start / stop reading sensor data periodically
    - If too many i2c errors in a row, assume the sensor is no longer connected and reset state machine
- Query sensor status from PC messages (indicates status of depth and IMU)


# Interface Scripts
- Change to work with ACK messages (once implemented)
- Support all new things


# Docs
- User Guide
    - Document robot coordinate system
    - Document each control mode (what it is, intended use cases, features, etc)
    - Document IMU configuration, options, and type of data
    - Document depth sensor configuration, options, and type of data
- Rewrite comm messages documentation to be easier to understand / read
- Document changes to communication at a message level (ACK messages) and recommended way to use / timeouts.


# Prototype Board
- Remove BNO280
- Add pullup resistors (10K) to SDA and SCL lines
- Add GPIO to reset IMU (BNO055) via pin


# PCB
- Make a PCB using off the shelf breakouts instead of using protobaord
