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
overlap_vectors = []
for r in range(np.size(dof_matrix, axis=0)):
    v = np.transpose(dof_matrix[r,:])
    overlap_vectors.append((np.matmul(dof_matrix, v) != 0).astype(int))


################################################################################
# Robot Orientation Information
################################################################################

# Current gravity vector from accelerometer. [0, 0, -1] is down
gravity_vector = np.array([0, 0, -1], dtype=np.double) * 9.81


################################################################################
# Target Motion Information
################################################################################

# Target motion in all 6 DoFs
target = np.transpose(np.matrix([
    #     x       y      z     pitch    roll     yaw
    [     0,      1,     1,      1,      1,       1    ]
], dtype=np.double))

# True if target vector above is relative to world (global target)
target_is_global = False


################################################################################
# Motor speed calculations
################################################################################
# In practice this would be repeated each time target changes
# If target is global this is also repeated each time gravity vector changes

if target_is_global:
    # TODO: Localize target
    raise Exception("Global targets not yet implemented.")

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
