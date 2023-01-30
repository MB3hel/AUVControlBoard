#!/usr/bin/env python3

import numpy as np



# ZYX (deg)
class Euler:
    def __init__(self, pitch: np.float32 = 0.0, roll: np.float32 = 0.0, yaw: np.float32 = 0.0):
        # In degrees
        self.pitch: np.float32 = pitch
        self.roll: np.float32 = roll
        self.yaw: np.float32 = yaw
    
    def __repr__(self) -> str:
        return "Euler(pitch = {}, roll = {}, yaw = {})".format(self.pitch, self.roll, self.yaw)
    
    def to_quaternion(self) -> 'Quaternion':
        ## Pitch about X, Roll about Y, Yaw about Z

        roll_rad = np.deg2rad(self.roll)
        pitch_rad = np.deg2rad(self.pitch)
        yaw_rad = np.deg2rad(self.yaw)

        cr = np.cos(roll_rad * 0.5)
        sr = np.sin(roll_rad * 0.5)
        cp = np.cos(pitch_rad * 0.5)
        sp = np.sin(pitch_rad * 0.5)
        cy = np.cos(yaw_rad * 0.5)
        sy = np.sin(yaw_rad * 0.5)

        q = Quaternion()
        q.w = cp * cr * cy + sp * sr * sy
        q.x = sp * cr * cy - cp * sr * sy
        q.y = cp * sr * cy + sp * cr * sy
        q.z = cp * cr * sy - sp * sr * cy

        return q


class Quaternion:
    def __init__(self, w: np.float32 = 0.0, x: np.float32 = 0.0, y: np.float32 = 0.0, z: np.float32 = 0.0):
        self.w: np.float32 = w
        self.x: np.float32 = x
        self.y: np.float32 = y
        self.z: np.float32 = z
    
    def __repr__(self) -> str:
        return "Quaternion(w = {}, x = {}, y = {}, z = {})".format(self.w, self.x, self.y, self.z)
    
    def to_euler(self) -> Euler:
        angles = Euler()

        # Pitch (about x axis)
        t0 = +2.0 * (self.w * self.x + self.y * self.z)
        t1 = +1.0 - 2.0 * (self.x * self.x + self.y * self.y)
        angles.pitch = np.arctan2(t0, t1)
     
        # Roll (about y axis)
        t2 = +2.0 * (self.w * self.y - self.z * self.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        angles.roll = np.arcsin(t2)
     
        # Yaw (about z axis)
        t3 = +2.0 * (self.w * self.z + self.x * self.y)
        t4 = +1.0 - 2.0 * (self.y * self.y + self.z * self.z)
        angles.yaw = np.arctan2(t3, t4)
     
        # Convert to degrees
        angles.pitch = np.rad2deg(angles.pitch)
        angles.roll = np.rad2deg(angles.roll)
        angles.yaw = np.rad2deg(angles.yaw)

        return angles
    
    def multiply_scalar(self, s: np.float32) -> 'Quaternion':
        return Quaternion(self.w * s, self.x * s, self.y * s, self.z * s)

    def divide_scalar(self, s: np.float32) -> 'Quaternion':
        return Quaternion(self.w / s, self.x / s, self.y / s, self.z / s)

    def multiply(self, second: 'Quaternion') -> 'Quaternion':
        res = Quaternion()
        res.w = self.w * second.w - self.x * second.x - self.y * second.y - self.z * second.z
        res.x = self.w * second.x + self.x * second.w + self.y * second.z - self.z * second.y
        res.y = self.w * second.y - self.x * second.z + self.y * second.w + self.z * second.x
        res.z = self.w * second.z + self.x * second.y - self.y * second.x + self.z * second.w
        return res

    def inverse(self) -> 'Quaternion':
        return self.conjugate().divide_scalar(self.magnitude())
    
    def conjugate(self) -> 'Quaternion':
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def magnitude(self) -> np.float32:
        return np.sqrt(self.w*self.w + self.x*self.x + self.y*self.y + self.z*self.z)
    
    def dot(self, second: 'Quaternion') -> np.float32:
        return self.w * second.w + self.x * second.x + self.y * second.y + self.z * second.z




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
curr_euler = Euler(90.0, 0.0, 0.0)


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
    cp = np.cos(np.deg2rad(curr_euler.pitch))
    sp = np.sin(np.deg2rad(curr_euler.pitch))
    cr = np.cos(np.deg2rad(curr_euler.roll))
    sr = np.sin(np.deg2rad(curr_euler.roll))
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
