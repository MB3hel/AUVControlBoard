#!/usr/bin/env python3

import numpy as np


################################################################################
# Thruster Configuration Information
################################################################################

# Motor Matrix Definition
motor_matrix = np.matrix([
    # MotorNum   x       y      z     pitch    roll     yaw
    [    1,     -1,     -1,     0,      0,      0,      +1    ],
    [    2,     +1,     -1,     0,      0,      0,      -1    ],
    [    3,     -1,     +1,     0,      0,      0,      -1    ],
    [    4,     +1,     +1,     0,      0,      0,      +1    ],
    [    5,      0,      0,    -1,     -1,     -1,       0    ],
    [    6,      0,      0,    -1,     -1,     +1,       0    ],
    [    7,      0,      0,    -1,     +1,     -1,       0    ],
    [    8,      0,      0,    -1,     +1,     +1,       0    ],
], dtype=np.double)

# Construct DoF Matrix and Motor Number Vector
dof_matrix = np.delete(motor_matrix, 0, axis=1)
motor_num_vec = motor_matrix[:,0]

# Construct overlap vectors for each motor
contribution_matrix = (dof_matrix != 0).astype(int)
overlap_vectors = []
for r in range(np.size(contribution_matrix, axis=0)):
    v = np.transpose(contribution_matrix[r,:])
    overlap_vectors.append((np.matmul(contribution_matrix, v) != 0).astype(int))


################################################################################
# Robot Orientation Information
################################################################################

# Current orientation in degrees (used in global mode)
curr_pitch = np.float32(90.0)
curr_roll = np.float32(0.0)
curr_yaw = np.float32(0.0)


################################################################################
# Target Motion Information
################################################################################

# Target motion in all 6 DoFs
target = np.transpose(np.matrix([
    #     x       y      z     pitch    roll     yaw
    [     0,      0,     -1,      0,      0,       0    ]
], dtype=np.double))
target_is_global = True


################################################################################
# Target localization
################################################################################

if target_is_global:
    cp = np.cos(np.deg2rad(curr_pitch))
    sp = np.sin(np.deg2rad(curr_pitch))
    cr = np.cos(np.deg2rad(curr_roll))
    sr = np.sin(np.deg2rad(curr_roll))
    R_x = np.matrix([
        [   1,      0,      0       ],
        [   0,      cp,     -sp     ],
        [   0,      sp,     cp      ]
    ], dtype=np.float32)
    R_y = np.matrix([
        [   cr,     0,      sr      ],
        [   0 ,     1,      0       ],
        [   -sr,    0,      cr      ]
    ], dtype=np.float32)
    R = np.matmul(R_y, R_x)
    target_1d = target.flatten().A1
    translation = target_1d[0:3].reshape(3, 1)
    rotation = target_1d[3:6].reshape(3, 1)
    translation = np.matmul(R, translation).flatten().A1
    rotation = np.matmul(R, rotation).flatten().A1
    target = np.concatenate((translation, rotation)).reshape(6, 1)


################################################################################
# Motor speed calculations
################################################################################
# In practice this would be repeated each time target changes

# Base speed calculation
speed_vec = np.matmul(dof_matrix, target)

# Scale motor speeds down as needed
while True:
    index = np.argmax(np.abs(speed_vec))
    m = np.abs(speed_vec[index].item())
    if m <= 1:
        break
    for i in range(np.size(overlap_vectors[index], axis=0)):
        if overlap_vectors[index][i] == 1:
            speed_vec[i] /= m


################################################################################
# Print Motor Speeds
################################################################################

for i in range(np.size(motor_num_vec, axis=0)):
    print("Motor {0}: {1:>4}%".format(int(motor_num_vec[i].item()), 
            int(speed_vec[i] * 100)))
