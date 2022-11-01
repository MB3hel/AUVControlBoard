# User Guide

This section describes the process of using the control board, **not** specific messages that are sent to achieve this process. The messages and how to send them are described on the [Communication Message](./comm_msgs.md) page.

## General Use Procedure

1. Connect to the control board (see [Communication Protocol](./comm_protocol)) section.
2. The control board does **not** save configuration. Thus you must configure it each time you connect to it. Configure all desired settings described in the Configuration section below. Minimally, make sure to select an operating mode.
3. Periodically, feed the motor watchdog (described below)
4. When needed, set a target motion for the used mode of operation. Note that you can reconfigure and change mode of operation whenever you want.
5. Periodically, read messages from the control board and handle the data they provide.


## LED Indicator

The ItsyBitsy M4 includes an RGB LED. This LED's color is used indicate operating mode, errors, or other information during firmware execution. Note that the LED is also used by the bootloader (red then green, followed by purple when the bootloader ends).

- Solid RED = Hardware fault occurred. Note that the system will reset after 2 seconds so this may appear to "blink"
- YELLOW = RAW mode
- BLUE-PURPLE = LOCAL mode
- ORANGE = GLOBAL mode
- CYAN = STABILITY_ASSIST mode


## Configuration

### Modes of Operation

**RAW Mode**:

In RAW mode, thruster speeds are directly controllable. Speeds (-1.0 to 1.0) for all 8 thrusters must be provided to the control board. It will then generate the PWM signals corresponding to those speeds.


**LOCAL Mode**:

In LOCAL mode, "speeds" in each of 6 degrees of freedom (DoF) are provided. This includes 3 translation speeds (x, y, z) and 3 rotation speeds (pitch, roll, yaw). In LOCAL mode, these DoFs are relative to the robot **not** the world (meaning they do not change as the robot's orientation changes).

Reference the coordinate system definition in the [math README](./math/README.md) for definitions of each DoF.

Some examples (assume robot is "level"):

- A target speed of 1.0 along y is "forward" motion at full possible speed
- A target speed of 1.0 along x and y is motion along 45 degree line (forward and right) at full possible speed
- A target speed of -0.5 yaw is rotation (clockwise when viewed from the top) at half speed
- A target speed of 0.75 yaw and 0.75 y is motion in a circle about the left edge of the robot (at 75% speed). The circle will be counterclockwise when viewed from the top.
- Adding -0.25 z to any above example adds a "submerge" of 25% to the resulting motion

In LOCAL mode, note that -1.0 z will always result in motion towards the "bottom" of the *robot* (which is only submerge if the robot is level). If the robot were pitched down 90 degrees (-90 degree pitch) a -1.0 z would result in the robot moving in "reverse" relative to the world. This same idea applies to all DoFs (because motion is relative to the *robot*).

### Thruster Inversions

TODO


### IMU Axis Configuration

TODO


## Motor Watchdog

The control board runs a "motor watchdog" which must be periodically "fed" to avoid an automatic shutoff of the motors. The watchdog can be fed in one of two ways.

1. A speed can be set in any mode as described above. When a new speed is set the watchdog will be "fed" automatically.
2. The watchdog can be explicitly "fed" using a "feed" command

The second method is useful when the desired motion has not changed. Setting desired motion (in any mode) results in some calculations occurring on the control board. To avoid performing these calculations more often than necessary, desired motion should only be set when it changes. As such, there needs to be a way to feed the watchdog without changing desired motion.

Generally, the computer interfacing with the control board should just periodically "feed" the watchdog using the explicit "feed" command. By feeding the watchdog often enough (recommended to feed between every 100 and 500 milliseconds), the motors will never be automatically disabled (unless something goes wrong).

The motor watchdog exists to ensure the motors will not continue running in unsafe situations such as

- Communication between control board and the computer is interrupted
- The computer is unable to provide speeds (program crashes, kernel panic, program deadlocks, etc)

The watchdog will automatically disable motors if it is not fed for 1500 milliseconds (1.5 seconds). When the watchdog kills motors, a message is sent to the PC to indicate that this has occurred.
