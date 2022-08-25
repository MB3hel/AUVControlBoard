#!/usr/bin/env python3

import numpy as np


class MotorManager:
    def __init__(self, motor_matrix, level_gravity_vector):
        # Store for later use
        self.level_gravity_vector = level_gravity_vector

        # Split and store motor matrix
        # dof matrix excludes motor numbers so it can be multiplied with target motion vector
        # dof_matrix = motor_matrix without first column
        self.dof_matrix = np.delete(motor_matrix, 0, axis=1)
        self.motor_nums = motor_matrix[:,0]

        # Find groups of motors that affect the same DOFs
        mask_matrix = (self.dof_matrix != 0).astype(int)
        mask_rows, indices = np.unique(mask_matrix, return_inverse=True, axis=0)
        mask_vector = indices.reshape(len(indices), 1)
        self.motor_groups = []
        for i in range(len(mask_rows)):
            self.motor_groups.append((mask_vector == i).astype(int))
    
    def calculate_speeds(self, local_target_vec):
        # Multiply to get motor output powers
        motor_speeds = np.matmul(self.dof_matrix, local_target_vec)

        # Scale down motor speeds within groups if needed
        scaled_motor_speeds = np.zeros(np.size(motor_speeds)).reshape(np.size(motor_speeds), 1)
        for i in range(len(self.motor_groups)):
            gp_speeds = np.multiply(motor_speeds, self.motor_groups[i])
            m = np.abs(gp_speeds).max()
            if m > 1:
                scaled_motor_speeds += gp_speeds / m
            else:
                scaled_motor_speeds += gp_speeds

        # Combine into nx2 matrix where n is number of motors
        motor_out = np.concatenate((self.motor_nums, scaled_motor_speeds), axis=1)

        # Matrix where first column is motor numbers and second column is motor speeds
        return motor_out

    def localize_translation(self, gravity_vector, world_translation_vec):
        # Gravity vector is in form [x, y, z] and is a unit vector
        # [0, 0, -1] is "normal" orientation with gravity
        # World translation vec is [x, y, z] speeds (NOT UNIT VECTOR)

        # Create rotation matrix from level gravity vector onto current gravity vector
        a = self.level_gravity_vector
        b = gravity_vector
        s = np.matrix((a + b).reshape(3, 1))
        st = s.transpose()
        R = (2 * np.matmul(s, st) / np.matmul(st, s)) - np.identity(3)
        print(R)
        

        # TODO: Calculate euler angles from rotation matrix (maybe not required)
        # TODO: Multiply by rotation matrix to rotate to be relative to robot and return vector
        

if __name__ == "__main__":
    # Defines how control board is oriented in the robot
    # This is what is measured as the gravity vector when the robot is level
    # This is a "world gravity vector"
    level_gravity_vector = np.array([0, 0, -1])

    # Matrix can be read as a table where each column is motor speeds to cause motion ONLY
    # in the specified DOF for the column (positive direction, full speed). 
    # X, Y, Z are translation and PITCH, ROLL, YAW are rotation
    # All motions are relative to robot position (not world position)
    motor_matrix = np.matrix([
        # Motor    +X      +Y      +Z   +PITCH   +ROLL   +YAW 
        [   1,     -1,     -1,      0,      0,      0,    -1    ],
        [   2,     +1,     -1,      0,      0,      0,    +1    ],
        [   3,     -1,     +1,      0,      0,      0,    +1    ],
        [   4,     +1,     +1,      0,      0,      0,    -1    ],
        [   5,      0,      0,     -1,     -1,     -1,     0    ],
        [   6,      0,      0,     -1,     -1,     +1,     0    ],
        [   7,      0,      0,     -1,     +1,     -1,     0    ],
        [   8,      0,      0,     -1,     +1,     +1,     0    ]
    ], dtype=np.double)
    manager = MotorManager(motor_matrix, level_gravity_vector)

    # Target DOF motions (as column vector)
    # Relative to ROBOT not WORLD
    # target = np.array(
    #     #   +X      +Y      +Z    +PITCH   +ROLL   +YAW
    #     [   1,      0,      0,      1,      1,      1   ],
    #     dtype=np.double,
    # )
    # target = target.reshape(len(target), 1)

    manager.localize_translation(level_gravity_vector, np.array([0, 0, 0]))

    # print(manager.calculate_speeds(target))
