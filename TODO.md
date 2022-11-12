# Firmware

## Bugs
- Either or both sensors seem to stop working after some duration (variable time, race cond)
    - Seems that sensors get stuck in a delay state (delay is started but delay done is never called)
    - This bug could also affect i2c0 timeout counter which is implemented the same way. This would be less likely to be seen since it would have to occur at the same time the i2c hardware "dies"
    - What seems to happen is the BNO055_DELAY_DONE or MS5837_DELAY_DONE flag gets set in IRQ handler (timers) as indicated by debug messages and LED. But, main never sees the flag set. Not sure how this happens. Have verified that memory clobbering around the flags_main field does not occur.
    - How else this can happen??? Flag get cleared somehow by something else???

## Control Modes / Command & Control
- Query commands for PID tuning
- Ensure provided arguments are in range (specifically speeds and speed limits for PID tune). If not, ignore (for now) or eventually, return an error code in ACK message.
- GLOBAL and STABILITY ASSIST modes should not work if required sensors are not connected
- Instead of applying speeds when receiving command, apply every x ms (fixed rate of speed sets)
    - Avoids issues where PIDs (SASSIST) are run too fast
    - Avoids issues where computer spamming setpoints can cause latency on sets due to control board not processing messages quickly enough due to performing motor math too often
- Implement PID controller
- Implement depth hold PID, pitch hold PID, and roll hold PID for stability assist mode.
- Implement yaw hold PID for stability assist mode.

## Communication
- Split into three types of messages. Do not impact encoding of messages, just makes documentation easier and defines what needs an ACK.
    - Set = computer telling control board to do something
    - Get = computer requesting information from control board
    - Status = Information sent from control board to computer unprompted
- All "set" messages respond with an ACK message (somehow indicating what the acknowledge is for). Should also include a status code.
- Should computer have to ACK status messages?
    - Maybe some should be (eg motor watchdog did kill), but others no (sensor data)
    - Alternatively, all status messages could be periodic and this can be ignored

## Sensors
- IMU (BNO055)
    - Unit select register should be configured by state machine
    - Configure axes via messages from PC
    - Command to read sensor data once
    - Command to start / stop reading sensor data periodically
    - Implement calibration status register read
    - Command to read calibration status from IMU
- Depth Sensor (MS5837)
    - Configure fluid density via messages from PC
    - Calibrate via messages from PC
    - Command to read sensor data once
    - Command to start / stop reading sensor data periodically


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
- Make a PCB using off the shelf breakouts instead of using protoboard
