# Using Control Board

TODO: This page is an outline. Update with details / actual contents.

- General Procedure
    - Connecting to Control Board
    - System configuration (motor matrix, thruster inversions, thruster PWM, reldof parameters)
    - Sensor configuration
    - Validating sensor connectivity
    - Modes of operation & setting speed
    - Motor Watchdog
    - Reading Sensor data
    - Example program using python interface script
- LED Indicator Info
- [Calibrate Sensors](./calibration.md)
- Custom interface code instead of python iface
    - Many users will want to build their own interface.
    - Python iface is a reference / an easy place to start, but not required
    - You just need to implement communication with the control board as described in comm protocol and messages pages
    - Note that users doing so may wish to read the section on SimCB and simulator too as supporting both TCP and UART in implementations can be useful.
