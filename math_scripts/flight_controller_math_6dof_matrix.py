#!/usr/bin/env python3

import numpy as np


def main():
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


    # dof matrix excludes motor numbers so it can be multiplied with target motion vector
    # dof_matrix = motor_matrix without first column
    dof_matrix = np.delete(motor_matrix, 0, axis=1)
    motor_nums = motor_matrix[:,0]

    # Find groups of motors that affect the same DOFs
    mask_matrix = (dof_matrix != 0).astype(int)
    mask_rows, indices = np.unique(mask_matrix, return_inverse=True, axis=0)
    mask_vector = indices.reshape(len(indices), 1)
    motor_groups = []
    for i in range(len(mask_rows)):
        motor_groups.append((mask_vector == i).astype(int))

    # Target DOF motions (as column vector)
    target = np.array(
        #   +X      +Y      +Z    +PITCH   +ROLL   +YAW
        [   1,      0,      0,      1,      1,      1   ],
        dtype=np.double,
    )
    target = target.reshape(len(target), 1)

    # Multiply to get motor output powers
    motor_speeds = dof_matrix * target

    # Scale down motor speeds within groups if needed
    scaled_motor_speeds = np.zeros(np.size(motor_speeds)).reshape(np.size(motor_speeds), 1)
    for i in range(len(motor_groups)):
        gp_speeds = np.multiply(motor_speeds, motor_groups[i])
        m = np.abs(gp_speeds).max()
        if m > 1:
            scaled_motor_speeds += gp_speeds / m
        else:
            scaled_motor_speeds += gp_speeds

    # Combine into nx2 matrix where n is number of motors
    motor_out = np.concatenate((motor_nums, scaled_motor_speeds), axis=1)

    # Print motor speeds
    print(motor_out)


if __name__ == "__main__":
    main()

