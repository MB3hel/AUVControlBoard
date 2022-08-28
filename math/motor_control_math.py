#!/usr/bin/env python3

import numpy as np



def skew(vector):
    """
    this function returns a numpy array with the skew symmetric cross product matrix for vector.
    the skew symmetric cross product matrix is defined such that
    np.cross(a, b) = np.dot(skew(a), b)

    :param vector: An array like vector to create the skew symmetric cross product matrix for
    :return: A numpy array of the skew symmetric cross product vector
    """
    if isinstance(vector, np.ndarray):
        return np.array([[0, -vector.item(2), vector.item(1)],
                         [vector.item(2), 0, -vector.item(0)],
                         [-vector.item(1), vector.item(0), 0]])
    else:
        return np.array([[0, -vector[2], vector[1]], 
                         [vector[2], 0, -vector[0]], 
                         [-vector[1], vector[0], 0]])



class MotorManager:
    def __init__(self, motor_matrix):
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

    def localize(self, gravity_vector, world_target):
        # Gravity vector is in form [x, y, z] (not unit vector)
        # [0, 0, -1] is "normal" orientation with gravity
        # Note: IMU used on control board allows axis remapping so firmware should
        #       always consider gravity to be -z when level
        # IMU axes can be remapped so x, y, and z axes are robot x, y, z not IMU board x, y, z
        # IMU also supports remapping axis signs as needed
        # World translation vec is [x, y, z] speeds (NOT UNIT VECTOR)

        # Reference: https://mwrona.com/posts/accel-roll-pitch/
        unit_gravity = gravity_vector / np.linalg.norm(gravity_vector)
        pitch = np.arcsin(unit_gravity[0])
        roll = np.arctan(unit_gravity[1] / unit_gravity[2])

        # Pitch rotation matrix (rotation about x axis)
        Rx = np.matrix([
            [1, 0, 0],
            [0, np.cos(pitch), -np.sin(pitch)],
            [0, np.sin(pitch), np.cos(pitch)]
        ])

        # Roll rotation matrix (rotation about y axis)
        Ry = np.matrix([
            [np.cos(roll), 0, np.sin(roll)],
            [0, 1, 0],
            [-np.sin(roll), 0, np.cos(roll)]
        ])

        # Ignoring heading (yaw) so rotation about z is zero so Rz = I
        # R = Rz*Ry*Rx = Ry*Rx
        R = np.matmul(Ry, Rx)

        orig_shape = np.shape(world_target)
        world_target = world_target.flatten()
        translation = world_target[0:3].reshape(3, 1)
        rotation = world_target[3:6].reshape(3, 1)

        ltranslation = np.matmul(R, translation)
        lrotation = np.matmul(R, rotation)

        return np.concatenate((ltranslation.A1, lrotation.A1)).reshape(orig_shape)
        

if __name__ == "__main__":
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
    manager = MotorManager(motor_matrix)

    # Target DOF motions (as column vector)
    # Relative to ROBOT not WORLD
    target = np.array(
        #   +X         +Y           +Z         +PITCH       +ROLL        +YAW
        [  0.15,     -0.26,       -0.36,          0,          0,           0   ],
        dtype=np.double,
    )
    target = target.reshape(len(target), 1)
    target_is_global = True

    # Current gravity vector ([0, 0, -1] is level robot). Only matters if target is global
    gravity_vector = np.array([0, 0, -1])

    if target_is_global:
        # TODO: Change gravity vector and make sure this is correct
        target = manager.localize(gravity_vector, target)

    print(manager.calculate_speeds(target))
