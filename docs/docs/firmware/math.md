# Control Math


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

<center>
![](./math_res/coord_system_1.png){: style="height:250px;"}
![](./math_res/coord_system_2.png){: style="height:250px;"}
![](./math_res/coord_system_rotations.png){: style="height:250px;"}
</center>

## Robot Local Coordinate System

- The robot's local coordinate system is defined as shown below where +y is forward, +x is right, and +z is up.
- Pitch and roll definitions remain the same as previously defined
    - +pitch raises front of the robot
    - +roll raises left of the robot
    - +yaw moves the front to the left

<center>
![](./math_res/robot_local_coord.png){: style="height:400px;"}
</center>

## Robot Thruster Arrangement

- The arrows on the diagrams below are the direction the thrusters push water when given a positive speed (meaning the force excreted on the robot is in the opposite direction).
- Note: Thruster numbers can be easily switched later if thrusters are connected to different pwm outputs.
- Note: Thruster directions can be inverted by the control board firmware if needed (without changing the math).

<center>
![](./math_res/thruster_config.png){: style="height:400px;"}
</center>


## 6DOF Motor Control Math

The following section covers math to calculate individual motor speeds to achieve the desired motion with a 6 degree of freedom system (6DoF = 3 translation and 3 rotation). The math remains valid for motor configurations where motion in some DoFs is not possible.

*[motor_control_math.py](./math_res/numpy_motor_math.py) is an implementation of the motor control math using numpy. It is used for prototyping / testing.*


### Motor Matrix

The motor matrix is generated based on physical frame and thruster configuration. The motor matrix associates motor numbers with their contributions to motion in different degrees of freedom.

The motor matrix can be thought of as a "table" where each column represents one degree of freedom and each row represents a motor. Each column is a set of motor speeds that cause motion **only** in the column's degree of freedom (and in the positive direction and at maximum possible speed). There are always 7 columns in the matrix. The number of rows is equal to the number of motors. Note that the motor numbers can be in any order (ie row order does not matter) whereas columns must be ordered as shown in the image below.

<center>
![](./math_res/motor_matrix.png){: style="height:250px;"}
</center>

To construct the motor matrix for a given thruster configuration, work one column at a time (after assigning motor numbers to rows). For each column determine the speeds for each motor to cause motion **only** in the positive direction of the column's DoF at the maximum possible speed. The motor speeds in each column must not cause motion in any other DoF (in an ideal scenario; in the real world things are never perfect). Note that all motor speeds are specified as a number between negative one and positive one.

The motor matrix for the thruster configuration shown above is as follows

<center>
![](./math_res/motor_matrix_sw8.png){: style="height:250px;"}
</center>


### DoF Matrix

The motor matrix is not directly used in calculations. Only a subset of it is. The DoF Matrix is a submatrix of the motor matrix excluding the first column (motor numbers). This matrix is used for all calculations. The motor number column is extracted and stored as a column vector and used to associate speed calculation results with the correct motor numbers after other calculations.

<center>
![](./math_res/dof_matrix_split.png){: style="height:250px;"}
</center>


### Local Targets

Next it is necessary to define a motion target, or a goal for the robot's motion in each DoF. For now, assume this target is relative to the robot, not the world (meaning the robot's orientation in space is irrelevant).

A motion target is a column vector where each row corresponds to a DoF (order matches order of DoFs in motor matrix columns). The value in each cell is a number from negative one to positive one and represents the target speed in each degree of freedom. The target can have motion in as many DoFs as desired. 

<center>
![](./math_res/target_vector.png){: style="height:250px;"}
</center>


This target motion vector can then be used to calculate individual motor speeds by multiplying it by the DoF matrix. The result of this multiplication is a column vector with as many entries as there are motors. Each entry is a motor speed. The motor speeds are in the same order as the motor number column vector (first column of motor matrix). This vector is the speed vector.

<center>
![](./math_res/motor_speed_calc.png){: style="height:250px;"}
</center>


Motion in multiple DoFs can be used to create any net motion the robot is capable of. For example, positive y translation and positive yaw of equal magnitudes (speeds) will result in the robot moving in a circle in the xy plane about the left edge of the robot (positive directions). The calculation for this scenario is shown below (at 100% speed).

<center>
![](./math_res/two_dof_calc.png){: style="height:250px;"}
</center>


Notice that the resultant motor speed vector has motor speeds that exceed 100% speed. This is because it is not possible to move at 100% speed in both of the specified DoFs at the same time. As such, motor speeds will need to be scaled down. This is discussed in the next section.


### Scaling Motor Outputs

The above example illustrates the need to scale down motor speeds. However, doing so is less trivial than it may initially appear.

The most intuitive option would be to divide all motor speeds by the largest magnitude (`m = max(abs(speed_vector))`) if `m` is larger than 1.0 (no need to divide if no value is larger than 1 because the motion is already possible). This solution works in the above example resulting in the following scaled speed vector

<center>
![](./math_res/scaled_speed_vec_all.png){: style="height:250px;"}
</center>

While this is the correct result for the above example, consider the following more complex example (where motion in more DoFs is added).

<center>
![](./math_res/many_dof_speed_calc.png){: style="height:250px;"}
</center>

If the previously described algorithm is applied `m = 3` which results in the following scaled speed vector

<center>
![](./math_res/scaled_all_many_dof.png){: style="height:225px;"}
</center>

However, this speed vector is scaled non-optimally. Notice that the maximum speed occurred at motor 5. However, motors 1, 2, 3, and 4 do not affect any of the same directions as motor 5. As such, it is not necessary to divide motor 1, 2, 3, 4 speeds by 3. Instead they should only be divided by 2 otherwise some DoF motions are slowed more than required (artificially reducing max speed).

In reality, it is only necessary to divide the speeds of some motors depending on where the max speed is located. If the max speed occurs at motor *i*, it is only necessary to divide the speed of any motors that "overlap" with motor *i*. Overlap is defined as sharing a contribution in any DoF. In terms of the DoF matrix, two motors *i* and *j* overlap if the row for motor *i* and the row for motor *j* have a non-zero entry in the same column for at least one column. Mathematically, this is easier to calculate if a contribution matrix is defined as "the dof matrix is not equal to zero". The contribution matrix is a "binary version" of the dof matrix, where any non-zero entry in the dof matrix becomes a 1 in the contribution matrix (and any zero remains a zero). 

<center>
![](./math_res/contribution_matrix_calc.png){: style="height:300px;"}
</center>


Then, in terms of the contribution matrix, two motors *i* and *j* overlap if the row for motor *i* and motor *j* have a one entry in the same column for at least one column. Mathematically, the number of shared non-zero entries is the dot product of the two rows.

To simplify later calculations an overlap vector will be generated for each motor in the dof matrix. The overlap vector is a vector of 1's and 0's indicating whether overlap occurs with the corresponding index motor in the speed vector. For motor i the overlap vector (`overlap_vec[i]`) is defined as "the product of the contribution matrix and the transpose of row `i` of the contribution matrix is not equal to zero". For example `overlap_vec[0]` is defined as follows

<center>
![](./math_res/overlap_vec_calc.png){: style="height:425px;"}
</center>

One overlap vector must be calculated for *each* motor. These are calculated ahead of time to reduce the number of operations that must be performed to calculate motor speeds (important when this is implemented on a microcontroller).

Finally, the following algorithm (described in pseudocode) is used to properly scale each motor. The scaling is done when no speed in the vector has a magnitude greater than 1.

```
while true
    // index is index in speed_vector at which m occurs
    m, index = max(abs(speed_vector))
    if m <= 1
        // No speeds exceed max magnitude, so done scaling
        break
    endif

    // Scale speed_vector as needed
    for i = 0; i < length(overlap_vector[index]); ++i
        if overlap_vector[index][i] == 1
            speed_vector[i] /= m;
        endif
    endfor
endwhile
```

Using this algorithm the earlier example results in the following scaled speed vector (which is optimal for the requested motions).

<center>
![](./math_res/proper_scaling.png){: style="height:250px;"}

*Motor 1, 2, 3, 4 speeds divided by 2 and motor 5, 6, 7, 8 speeds divided by 3. This results in the fastest motor within each group being at 100% speed, thus this is optimal scaling.*
</center>


### Global Targets

Instead of providing desired motion relative to the robot's orientation, it is often easier to specify motion relative to the world (at least partially). This requires knowing information about the robot's orientation in 3D space. However, for this application the robot's heading will be ignored (meaning x and y are relative to the robot's orientation, but z is world-relative). 

This effectively turns the target vector previously provided into a pseudo world-relative motion target (DoFs are world coordinate system DoFs not robot coordinate system DoFs). *However, y still means forward relative to robot heading **not** relative to the world coordinate system (same idea for x too).*

This method is used instead of a true global target for two reasons
- The method described above does not require knowing the robot's heading in 3D space. As such, the required information can be entirely obtained using an accelerometer. No use of gyroscope or magnetometer is required. This is beneficial as gyroscopes drift and magnetometers become unreliable in close proximity to motors.
- Missing code's knowledge of the robot's position relative to objects of interest often has no knowledge of a world coordinate system. As such, keeping x and y translations robot-relative simplifies mission code and reduces errors for closed loop control in mission code.

The target vector can be split into two parts: a translation vector and a rotation vector.

<center>
![](./math_res/target_vector_split.png){: style="height:250px;"}
</center>


Both vectors are in an [x, y, z] order.
- Translations are along the given axes
- Rotations are about the given axes

The idea is to determine a rotation matrix to translate the world gravity vector to the robot's measured gravity vector. This is the same rotation that should then be applied to each of the vectors described above (translation and rotation).

It is assumed that when the robot's coordinate frame matches the world's coordinate frame, the measured gravity vector will be [0, 0, -g] (meaning in the negative z direction). This must be configured to be the case (IMU supports axis remapping internally to allow this regardless of how the IMU is mounted). Then, given a world gravity vector (`g_w`) and a measured gravity vector `g_r` a rotation matrix (`R`) to rotate vectors from the world coordinate system into the robot's coordinate system can be calculated as shown below

<center>
![](./math_res/rotation_matrix_calc.png){: style="height:100px;"}
</center>

Where `[v_c]_x` is the skew symmetric cross product matrix of `v_c` defined as follows

<center>
![](./math_res/skew_symmetric.png){: style="height:100px;"}
</center>

Then each of the translation and rotation targets can be rotated by multiplication by the rotation matrix (`R`). The translation and rotation vectors are then concatenated to create the full local target vector.

<center>
![](./math_res/apply_rotation.png){: style="height:100px;"}
</center>


## Stability Assist Mode Math

Stability assist mode is used to abstract a "2D" working plane to control the vehicle in. This is done by using closed-loop control to maintain pitch, roll, and depth in 3D space (leaving the x, y, and yaw DoFs). Optionally, closed-loop control can also be used for yaw leaving just the x and y DoFs.

This is built on top of global mode. Thus, stability assist mode calculates the current desired speed in each degree of freedom managed by closed loop controllers (pitch, roll, depth, and sometimes yaw). The speeds in other DoFs are specified by the PC just as they would be in global mode.


### Orientation Closed-Loop Control

*[orientation_math.py](./math_res/orientation_math.py) is an implementation of the orientation closed-loop math using numpy. It is used for prototyping / testing.*

Separate PID controllers are used to track the target pitch, roll, and yaw. This allows the output of each PID to be used as a "speed" for one DoF for global mode.

The target (desired) orientation is specified using Euler angles (3-2-1, ZYX), however it will be converted to a Quaternion before use. The use of euler angles allows easily "ignoring" yaw in the event that only pitch and roll are to be managed by closed-loop controllers.

The current angle is read from the IMU (as a quaternion due to bugs in some versions of the BNO055 firmware with its euler angles). It is converted to euler angles (ZYX) additionally. The target orientation is specified using euler angles (ZYX) from the PC. If yaw is not managed by closed-loop control, the current yaw from the IMU reading is copied into the target euler angles (replacing the target yaw). This ensures that the yaw difference between the target and current angle will be zero.

Next, the target is converted to a quaternion. The angle between the current and target quaternion is then calculated. This is done using the following

$q_\textrm{diff} = (q_\textrm{target}) (q_\textrm{current})^{-1}$

However, this is not guaranteed to be the minimal angle between the current and target quaternions. If the dot product of the two quaternions is negative, the following equation is instead used to get the minimum angle.

$q_\textrm{diff} = (q_\textrm{target}) (-q_\textrm{current})^{-1}$


This minimum angle quaternion ($q_\textrm{diff}$) is then converted to euler angles. The pitch, roll, and yaw of the euler angles are used as the current error for the pitch, roll, and yaw closed-loop PID controllers.

![](./math_res/construct_orientation_errors.png)


### Depth Closed-Loop Control

Depth closed-loop control is implemented using a PID. This PID's output is used as the target speed in the z DoF. The error is the difference between the current depth sensor reading and the specified target depth (in meters; negative for below the surface).


## IMU Angle Accumulation

*[accumulate_angles.py](./math_res/accumulate_angles.py) is an implementation of the math used to determine the difference between two quaternions (used to determine change in euler angles for accumulation).*

The euler and quaternion values provided by the IMU are not directly useful for tracking multiple rotations of the vehicle. Unlike simply integrating gyroscope data, euler angles (pitch, roll, yaw) and quaternions do not track the number of times the vehicle has rotated about a particular axis.

While integrating raw gyro data would provide this, such a solution would not be rotations about the robot's axes, not the world's axes (gyro z of 500 does not necessarily mean the robot has yawed 500 degrees; the robot could have been oriented at a pitch of 90).

To address this, it is necessary to track changes between subsequent quaternions from the IMU. Quaternions are used for three reasons

1. Firmware bugs in some BNO055 firmware versions lead to unstable euler angles
2. It is mathematically easier to determine the "shortest rotation" between two quaternions
3. Quaternions still allow accumulation to work in "gimbal lock" situations

The idea is to compare each quaternion read from the IMU with the previous quaternion received from the IMU (note that quaternions of all zeros are ignored to avoid issues with invalid IMU data after exiting config mode). For each read quaternion:

- Calculate the shortest set of rotations from the previous to the current quaternion
- Convert the shortest angle to euler angles
- Add the pitch, roll, and yaw from the shortest euler angles to accumulated pitch, roll, and yaw variables
- Note that if the IMU axis config changes, the accumulated data should be zeroed and the previously read quaternion discarded.

This method makes the assumption that the smallest rotation between two quaternions is the most probable path the robot took to change its orientation. This is an approximation, however it is a fairly good one as long as sample rate of data is sufficiently high. This method of approximation effectively liberalizes motion between two rotations. The specifics of the path are lost. However, if the sample rate is high enough, the length of the path is sufficiently small that this is a good approximation.

The second "issue" with this approximation has to do with rotations exceeding 180 degrees. The method for determining shortest path between two quaternions will be incorrect if the vehicle rotates more than 180 degrees in any axis (because the shortest path would have involved rotating the other direction). To guarantee rotations between two samples never exceed 180 degrees, the max measured rotation rate of the gyro is considered. For the BNO055 this is 2000 degrees per second. Thus, with a sample period of $l$ milliseconds, the largest angle change between samples is

$\frac{2000 \textrm{ deg}}{1 \textrm{ sec}} \cdot \frac{1 \textrm{ sec}}{1000 \textrm{ ms}} \cdot \frac{l \textrm{ ms}}{1 \textrm{ sample}} = 2l \textrm{ deg / sample}$  

To ensure that changes of more than 180 degrees do not occur, the following must be satisfied

$2l < 180 \rightarrow l < 90 \textrm{ milliseconds}$

However, it is possible for some samples from the IMU to be delayed (ie I2C bus busy with another sensor) or lost (I2C failure). Thus, it is necessary to choose a value for $l$ that allows for at least one sample to be lost. When a sample is lost, this doubles the effective time between samples. Thus, it is necessary to half $l$

$l < 45 \textrm{ milliseconds}$

By further reducing $l$ it is possible to allow for larger delays or more lost samples. The current firmware samples IMU data every 15ms (the max rate supported by the BNO055 in fusion mode is 100Hz = 10ms period). Using $l=15 \textrm{ ms}$ it is possible for 5 consecutive samples to be lost while still guaranteeing that no more than 90ms passes between valid samples (thus still ensuring no more than 180 degree change between samples).