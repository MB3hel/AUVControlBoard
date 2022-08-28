# AquaPack Robotics Control Board Math

`motor_control_math.py` is an implementation of the motor control math using numpy. It is used for prototyping / testing.

`motor_control_math.c` is a pure c implementation of the required math (can be used directly in embedded firmware later). *DOES NOT EXIST YET*


## Coordinate System Definition

- The coordinate system is defined by the images below
- Pitch is defined as rotation about the x-axis
- Roll is defined as rotation about the y-axis
- Yaw is defined as rotation about the z-axis
- Positive pitch, roll, and yaw are defined by the right hand rule
    - Point your right thumb in the positive direction of the axis being rotated about. The fingers of the hand curve in the direction of positive rotation. [Reference](https://en.wikipedia.org/wiki/Right-hand_rule)
    - Positive pitch is defined as counter-clockwise rotation in the yz plane when view from the +x side
    - Positive roll is defined as counter-clockwise rotation in the xz plan when view from the +y side
    - Positive yaw is defined as counter-clockwise rotation in the xy plane when viewed from the +z side

<p align="center">
    <img height="250" src="./img/coord_system_1.png">
    <img height="250" src="./img/coord_system_2.png">
    <img height="250" src="./img/coord_system_rotations.png">
</p>


## Robot Local Coordinate System

- The robot's local coordinate system is defined as shown below where +y is forward, +x is right, and +z is up.
- Pitch and roll definitions remain the same as previously defined
    - +pitch raises front of the robot
    - +roll raises left of the robot
    - +yaw moves the front to the left

<p align="center">
    <img height="400" src="./img/robot_local_coord.png">
</p>


## Robot Thruster Arrangement

- The arrows on the diagrams below are the direction the thrusters push water when given a positive speed (meaning the force excreted on the robot is in the opposite direction).
- Note: Thruster numbers may not match SW8 exactly. This can be addressed later by adjusting motor numbers in the motor matrix (will be explained later).
- Note: Directions of arrows may be incorrect for positive speed on some thrusters. Inverting sign of speed per motor as needed in software can correct this.

<p align="center">
    <img height="400" src="./img/thruster_config.png">
</p>


## 6DOF Motor Math

### Motor Matrix

TODO

### Local Targets

TODO


### Scaling Motor Outputs

TODO


### Global Targets

TODO
