# AquaPack Robotics Control Board

The control board is a replacement for the Cube Orange / Pixhawk running ArduSub that was originally used on SeaWolf VIII. The control board was developed to address continuous issues with both control and sensors when using either off the shelf autopilot.

The control board is designed using development boards and breakout boards to reduce development time and simplify maintenance / modifications later. This is not a size-optimized solution (and does not need to be).

The control board generates ESC control signals (PWM), acquires and processes sensor data (using onboard IMU / IMUs, depth sensor, and sensor fusion / filtering algorithms) as well as running control loops for system motion and stability control. Additionally, an interface to the computer (Jetson) is provided to acquire sensor data and allow control of motors using high or low level methods (vectored motion or direct control of each motor’s speed).


## Repository Structure

- `firmware`: PlatformIO project and source code for the control board firmware (Microchip ASF4 based)
- `hwtest`: PlatformIO project and source code for an Arduino-based test program to verify hardware functionality.
- `img`: Images used in documentation
- `math`: Demo of thruster control math as well as documentation of the math
- `prototype`: Assembly instructions for control board prototype
- `scripts`: Python scripts that demo interfacing with the control board


## Firmware Development Environment

Both the [control board firmware](./firmware/) and the [hardware test](./hwtest/) program are developed using [PlatformIO](https://platformio.org/). PlatformIO is installed as an extension to [VSCode](https://code.visualstudio.com/).


## Flashing a New Board

1. [Update the U2F Bootloader](https://learn.adafruit.com/introducing-adafruit-itsybitsy-m4/update-the-uf2-bootloader)
    - Press the reset button twice to enter bootloader mode
    - Copy the downloaded bootloader `.u2f` file to the `ITSYM4BOOT` drive

2. Upload the Hardware Test Program (`hwtest`)
    - Open the `hwtest` folder in VSCode (after installing platformio extension)
    - Upload the project using the upload button on the toolbar
    - The port should be automatically detected. If not, [specify it](https://docs.platformio.org/en/latest/projectconf/section_env_upload.html) in `platformio.ini`
    - The hardware test will not begin until a serial connect is opened with the board. Open the serial monitor and make sure all tests pass. The red LED will turn off when successful, or blink on failure.

3. Upload the firmware
    - Open the `firmware` folder in VSCode (after installing platformio extension)
    - Upload the project using the upload button on the toolbar
    - The port should be automatically detected. If not, [specify it](https://docs.platformio.org/en/latest/projectconf/section_env_upload.html) in `platformio.ini`


## Control Board Usage

This section describes the process of using the control board, **not** specific messages that are sent to acheive this process. The messages and how to send them are described in the "Communication Protocol" section.

### Usage Examples

Python scripts demonstrating use of the control board are available in the [scripts](./scripts) folder.

### General Configuration

TODO: MODE, TINV


### Modes of Operation

#### RAW Mode

In RAW mode, thruster speeds are directly controllable. Speeds (-1.0 to 1.0) for all 8 thrusters must be provided to the control board. It will then generate the PWM signals cooresponding to those speeds.


#### LOCAL Mode

In LOCAL mode, "speeds" in each of 6 degrees of freedom (DoF) are provided. This includes 3 translation speeds (x, y, z) and 3 rotation speeds (pitch, roll, yaw). In LOCAL mode, these DoFs are relative to the robot **not** the world (meaning they do not change as the robot's orientation changes).

Reference the coordinate system definition in the [math README](./math/README.md) for definitions of each DoF.

Some examples (assume robot is "level"):

- A target speed of 1.0 along y is "forward" motion at full possible speed
- A target speed of 1.0 along x and y is motion along 45 degree line (forward and right) at full possible speed
- A target speed of -0.5 yaw is rotation (clockwise when viewed from the top) at half speed
- A target speed of 0.75 yaw and 0.75 y is motion in a circle about the left edge of the robot (at 75% speed). The circle will be counterclockwise when viewed from the top.
- Adding -0.25 z to any above example adds a "submerge" of 25% to the resulting motion

In LOCAL mode, note that -1.0 z will always result in motion toawrds the "bottom" of the *robot* (which is only submerge if the robot is level). If the robot were pitched down 90 degrees (-90 degree pitch) a -1.0 z would result in the robot moving in "reverse" relative to the world. This same idea applies to all DoFs (because motion is realative to the *robot*).


### Motor Watchdog

The control board runs a "motor watchdog" which must be periodically "fed" to avoid an automatic shutoff of the motors. The watchdog can be fed in one of two ways.

1. A speed can be set in any mode as described above. When a new speed is set the watchdog will be "fed" automatically.
2. The watchdog can be explicitly "fed" using a "feed" command

The second method is useful when the desired motion has not changed. Setting desired motion (in any mode) results in some calculations occuring on the control board. To avoid perfoming these calculations more often than necessary, desired motion should only be set when it changes. As such, there needs to be a way to feed the watchdog without changing desired motion.

Generally, the computer interfacing with the control board should just periodically "feed" the watchdog using the explicit "feed" command. By feeding the watchdog often enough (recomended to feed between every 100 and 500 milliseconds), the motors will never be automatically disabled (unless something goes wrong).

The motor watchdog exists to ensure the motors will not continue running in unsafe situations such as

- Communication between control board and the computer is interrupted
- The computer is unable to provide speeds (program crashes, kernel panic, program deadlocks, etc)

The watchdog will automatically disable motors if it is not fed for 1500 milliseconds (1.5 seconds). When the watchdog kills motors, a message is sent to the PC to indicate that this has occured.


### Communication Protocol

Communication with the control board is defined by three layers
- Hardware communication layer: How bytes are sent at a hardware level and how this is abstracted by a PC
- Message format and construction: How arbitrary data is composed into an identifyable message
- Command / message set: What the data of a message should actually be to perform different tasks


#### Hardware Communication Layer

Messages are sent to the control board over the ItsyBitsy M4's builtin USB port. The control board acts as a USB ACM CDC device. In practice, this means that it shows up as a serial (UART) port on the computer it is connected to. However, baud rate settings are irrelevant (and changing baud rates has no effect). As such, messages are sent to / received from the control board using "UART" with an undefined baud rate (most software stacks require selecting any baud rate; it will not be applied lower level).


#### Message Format and Construction

The messages sent to / received from the control board have a specific format. Each message is a raw set of bytes (unsigned byte array). This set of bytes is the "payload data" of the message. The "payload data" is the data that is contained within a single message.

To be able to identify what data is part of a single message, it is necessary to add some additional information around the payload. The control board uses a special byte to indicate the start of a message (`START_BYTE`) and another one to identify the end of a message (`END_BYTE`). 

Since the payload could itself contain a start or end byte, there is also an escape byte (`ESCAPE_BYTE`) used to escape a `START_BYTE`, `END_BYTE`, or an `ESCAPE_BYTE` in the payload. 
- `START_BYTE` becomes `ESCAPE_BYTE`, `START_BYTE`
- `END_BYTE` becomes `ESCAPE_BYTE`, `END_BYTE`
- `ESCAPE_BYTE` becomes `ESCAPE_BYTE`, `ESCAPE_BYTE`

This is similar to escaping a quote in a string using a backslash.

For the control board:
- `START_BYTE` = 253 (unsigned 8-bit) = -3 (signed 8-bit)
- `END_BYTE` = 254 (unsigned 8-bit) = -2 (signed 8-bit)
- `ESCAPE_BYTE` = 255 (unsigned 8-bit) = -1 (signed 8-bit)

Additionally, each message contains a 16-bit CRC for the payload data (CCITT-FALSE algorithm). This CRC is calculated on the original (unescaped) payload data (no start byte, end byte, etc). The CRC is appended (big endian) to the message just after the payload (just before `END_BYTE`). When the other side receives the message it can calculate the CRC of the received payload and compare it to the received crc. If the crc values match, the message is not corrupt.

<p align="center">
    <img height="175" src="./img/cb_msg_construction.png">
</p>


### Commands and Messages

The following are the messages sent to the control board or received from the control board and what they mean / do. These messages are the *payload* in the message format described above. Note that all characters are ASCII encoded unless enclosed within square brackets. A literal square bracket is escaped with a backslash. A literal backslash is also escaped with a backslash. Tokens inside square brackets are described per message below.


**Set MODE**

*Sent from computer to control board*

```
MODE[mode_val]
```

where `mode_val` is one of the following

- `R`: RAW mode
- `L`: LOCAL mode


**Get MODE**

*Sent from computer to control board*

```
?MODE
```

Response (from control board to computer) will be in the format

```
MODE[mode_val]
```

where `mode_val` is one of the following

- `R`: RAW mode
- `L`: LOCAL mode


**Set Thruster Inversions**

*Sent from computer to control board*

```
TINV[invert1][invert2][invert3][invert4][invert5][invert6][invert7][invert8]
```

where `invertx` (for `x` = 1, 2, ..., 8) is a 1 or a zero (8-bit integer) where 1 means thruster `x` is inverted and 0 means thruster `x` is not inverted.


**Get Thruster Inversions**

*Sent from computer to control board*

```
?TINV
```

Response (from control board to computer) will be in the format

```
TINV[invert1][invert2][invert3][invert4][invert5][invert6][invert7][invert8]
```

where `invertx` (for `x` = 1, 2, ..., 8) is a 1 or a zero (8-bit integer) where 1 means thruster `x` is inverted and 0 means thruster `x` is not inverted.


**Set RAW Speeds**

*Sent from computer to control board*

```
RAW[speed1][speed2][speed3][speed4][speed5][speed6][speed7][speed8]
```

where `speedx` (for `x` = 1, 2, ..., 8) is a little endian encoded 32-bit float between -1.0 and 1.0 indicating the speed of thruster `x`.


**Feed Motor Watchdog**

*Sent from computer to control board*

```
WDGF
```


**Motor Watchdog Did Disable**

*Sent from control board to computer if the motor watchdog automatically kills motors.*

```
WDGK
```
