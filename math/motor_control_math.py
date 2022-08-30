#!/usr/bin/env python3

import numpy as np


class MotorManager:
    def __init__(self, motor_matrix):
        # Split and store motor matrix
        # dof matrix excludes motor numbers so it can be multiplied with target motion vector
        # dof_matrix = motor_matrix without first column
        self.dof_matrix = np.delete(motor_matrix, 0, axis=1)
        self.motor_nums = motor_matrix[:,0]

        # Find overlap in motors (used for scaling motor speeds)
        self.overlap_vectors = []
        for r in range(np.size(self.dof_matrix, axis=0)):
            row = self.dof_matrix[r,:]
            v = np.transpose(row)
            vec = np.matmul(self.dof_matrix, v)
            self.overlap_vectors.append((vec != 0).astype(int))
    
    def calculate_speeds(self, local_target_vec, deadband=1e-6):
        # Not allowed to be negative
        deadband = np.abs(deadband)

        # Multiply to get motor output powers
        motor_speeds = np.matmul(self.dof_matrix, local_target_vec)

        # Scale down motor speeds as needed
        while True:
            index = np.argmax(np.abs(motor_speeds))
            m = np.abs(motor_speeds[index].item())
            if m <= 1:
                break
            for i in range(np.size(self.overlap_vectors[index])):
                if self.overlap_vectors[index][i]:
                    motor_speeds[i] /= m
        
        # Apply deadband
        for i in range(np.size(motor_speeds, axis=0)):
            if np.abs(motor_speeds[i, 0]) < deadband:
                motor_speeds[i, 0] = 0
                
        # Combine into nx2 matrix where n is number of motors
        motor_out = np.concatenate((self.motor_nums, motor_speeds), axis=1)

        # Matrix where first column is motor numbers and second column is motor speeds
        return motor_out

    def localize_target(self, gravity_vector, world_target):
        # Gravity vector is in form [x, y, z] (not unit vector)
        # [0, 0, -1] is "normal" orientation with gravity (may be scaled by "g")
        # Note: IMU used on control board allows axis remapping so firmware should
        #       always consider gravity to be -z when level
        # IMU axes can be remapped so x, y, and z axes are robot x, y, z not IMU board x, y, z
        # IMU also supports remapping axis signs as needed

        pitch = np.arctan2(-gravity_vector[1], gravity_vector[2])
        roll = np.arctan2(gravity_vector[0], np.sqrt(np.power(gravity_vector[1], 2) + np.power(gravity_vector[2], 2)))

        print(pitch)
        print(roll)

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
        # R = Rz*Ry*Rx = Ry*Rx   (net rotation matrix)
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
        [   0,          1,           1,          1,          1,           1   ],
        dtype=np.double,
    )
    target = target.reshape(len(target), 1)
    target_is_global = False

    # Current gravity vector ([0, 0, -1] is level robot). Only matters if target is global
    gravity_vector = np.array([0, 0, -1])

    if target_is_global:
        # TODO: Make sure this actually works right by varying gravity vector (3d space is hard...)
        # TODO: Some negative signs may be wrong. Need to actually wrap my head around coord system...
        target = manager.localize_target(gravity_vector, target)

    print(manager.calculate_speeds(target))
