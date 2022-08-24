Note: Control board has three modes of operation
    - RAW: Jetson provides raw motor speed commands (set motor x to speed y)
    - MANUAL_LOCAL: Jetson provides target motion vector (relative to robot frame)
    - MANUAL_GLOBAL: Jetson provides target motion vector (relative to world frame)
    - STABILITY_ASSIST: TODO: Determine exactly what this would look like.


- generate_target_vector.py takes information about target that would be given to control board by jetson (in stability assist mode) and info that would be read from sensors and uses it to calculate a target motion vector (robot local)
- dof_target_to_motor_speeds.py takes a target motion vector (6 dof) and uses the motor matrix (in this file) to generate motor speeds to achieve this motion. The target motion vector must be relative to the robot's frame (ie -z moves down in the world only if robot is "level")