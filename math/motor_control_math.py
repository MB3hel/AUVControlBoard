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

# Orientation information ([pitch, roll, yaw]])
orientation_pry = np.array([0, 0, 90], dtype=np.double) * np.pi / 180.0

# Orientation information (quaternion [w, x, y, z]) used in calculations
# Note: Imu provides this directly
orientation = np.array([0, 0, 0, 0], dtype=np.double)

# See https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
cy = np.cos(orientation_pry[2] * 0.5)
sy = np.sin(orientation_pry[2] * 0.5)
cp = np.cos(orientation_pry[0] * 0.5)
sp = np.sin(orientation_pry[0] * 0.5)
cr = np.cos(orientation_pry[1] * 0.5)
sr = np.sin(orientation_pry[1] * 0.5)
orientation[0] = cr * cp * cy + sr * sp * sy
orientation[1] = sr * cp * cy - cr * sp * sy
orientation[2] = cr * sp * cy + sr * cp * sy
orientation[3] = cr * cp * sy - sr * sp * cy


################################################################################
# Target Motion Information
################################################################################

# Target motion in all 6 DoFs
target = np.transpose(np.matrix([
    #     x       y      z     pitch    roll     yaw
    [     0,      1,     0,      0,      0,       0    ]
], dtype=np.double))
target_is_global = True


################################################################################
# Target localization
################################################################################

# See https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
# See https://en.wikipedia.org/wiki/Quaternion#Multiplication_of_basis_elements
# See https://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/code/index.htm
def quaternion_multiply(q1, q2):
    # res, q1, and q2 in order [w, x, y, z]
    res = np.array([0, 0, 0, 0], dtype=np.double)
    res[0] = -q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3] + q1[0] * q2[0]
    res[1] =  q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2] + q1[0] * q2[1]
    res[2] = -q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1] + q1[0] * q2[2]
    res[3] =  q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0] + q1[0] * q2[3]
    return res

if target_is_global:
    # Translation vector is a vector of speeds in each axis
    # However, it can be interpreted as a point vector where the distance
    # from each point to the origin is the speed in the corresponding axis
    # Note: the "w" element is populated with  zero
    translation_vec = np.array([0, target[0].item(), target[1].item(), target[2].item()])

    # Similar idea for rotation vector
    rotation_vec = np.array([0, target[3].item(), target[4].item(), target[5].item()])

    # Conjugate of orientation quaternion
    orientation_conj = np.array([
         orientation[0], 
        -orientation[1], 
        -orientation[2], 
        -orientation[3]
    ])

    # Convert to local target
    translation_vec = quaternion_multiply(quaternion_multiply(orientation, translation_vec), orientation_conj)
    rotation_vec = quaternion_multiply(quaternion_multiply(orientation, rotation_vec), orientation_conj)

    # Combine target info into one vector again
    target = np.concatenate((translation_vec[1:4], rotation_vec[1:4]), axis=0).reshape(6, 1)
    print(target)

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
