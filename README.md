# AquaPack Robotics Control Board

The control board is a replacement for the Cube Orange / Pixhawk running ArduSub that was originally used on SeaWolf VIII. The control board was developed to address continuous issues with both control and sensors when using either off the shelf autopilot.

The control board is designed using development boards and breakout boards to reduce development time and simplify maintenance / modifications later. This is not a size-optimized solution (and does not need to be).

The control board generates ESC control signals (PWM), acquires and processes sensor data (using onboard IMU / IMUs, depth sensor, and sensor fusion / filtering algorithms) as well as running control loops for system motion and stability control. Additionally, an interface to the computer (Jetson) is provided to acquire sensor data and allow control of motors using high or low level methods (vectored motion or direct control of each motor’s speed).


TODO: Documentation on interfacing with control board (communication protocol, available PIDs, control modes, etc)


## Flashing a New Board

- *Note: to enter bootloader mode pres the reset button twice quickly. This is required before programming the board.*
- [Update U2F Bootloader](https://learn.adafruit.com/introducing-adafruit-itsybitsy-m4/update-the-uf2-bootloader)
- Upload the test program
- Upload the firmware
