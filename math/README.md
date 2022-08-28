# AquaPack Robotics Control Board Math

`motor_control_math.py` is an implementation of the motor control math using numpy. It is used for prototyping / testing.

`motor_control_math.c` is a pure c implementation of the required math (can be used directly in embedded firmware later). *DOES NOT EXIST YET*


## Coordinate System Definition

- The coordinate system defined by the image below is used to define axis relations (axis arrows in positive directions)
- Pitch is defined as rotation about the x-axis
- Roll is defined as rotation about the y-axis
- Yaw is defined as rotation about the z-axis
- Positive pitch, roll, and yaw are defined by the right hand rule
    - Point your right thumb in the positive direction of the axis being rotated about. The fingers of the hand curve in the direction of positive rotation. [Reference](https://en.wikipedia.org/wiki/Right-hand_rule)
    - Positive pitch is defined as counter-clockwise rotation in the yz plane when view from the +x side
    - Positive roll is defined as counter-clockwise rotation in the xz plan when view from the +y side
    - Positive yaw is defined as counter-clockwise rotation in the xy plane when viewed from the +z side

<div style="text-align: center;">

![](./img/coord_system_1.png)

![](./img/coord_system_2.png)

![](./img/coord_system_rotations.png)

</div>


## Robot Local Coordinate System

- The robot's local coordinate system is defined as shown below where +y is forward, +x is right, and +z is up.
- Pitch and roll definitions remain the same as previously defined
    - +pitch raises front of the robot
    - +roll raises left of the robot
    - +yaw moves the front to the left

<div style="text-align: center;">

![](./img/robot_local_coord.png)

</div>


## Robot Thruster Arrangement

TODO: Make sure this matches actual thruster configuration of SW8 (note: arrows in direction thruster moves water when given positive power).

Note: Directions don't actually have to match robot as long as thrusters can be individually inverted to match expected diagram.


## 6DOF Motor Math

### Motor Matrix

TODO

### Local Targets

TODO


### Scaling Motor Outputs

TODO


### Global Targets

TODO
