# AquaPack Robotics SW8 Control Board Documentation

The control board is a replacement for the Cube Orange / Pixhawk running ArduSub that was originally used on SeaWolf VIII. The control board was developed to address continuous issues with both control and sensors when using either off-the-shelf autopilot.

The control board generates ESC control signals (PWM), acquires and processes sensor data (using onboard IMU / IMUs, depth sensor, and sensor fusion / filtering algorithms) as well as running control loops for system motion and stability control. Additionally, an interface to the computer is provided to acquire sensor data and allow control of motors using high or low level methods.

## [User Guide](./user_guide.md)

## [Communication Protocol](./comm_protocol.md)

## [Communication Messages](./comm_msgs.md)
