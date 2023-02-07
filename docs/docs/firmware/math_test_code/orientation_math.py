
import numpy as np
from typing import Tuple
from util import Quaternion


################################################################################
# Convert euler to quaternion
################################################################################

curr_quat = Quaternion.from_pry(0, 0, 0)
target_quat = Quaternion.from_pry(0, 0, 0)


################################################################################
# Calculate shortest angle between curr and target
################################################################################

rot_quat = target_quat.multiply(curr_quat.inverse())
rot_euler = rot_quat.to_pry()


print("Quat: {}".format(rot_quat))
print()
# angle, axis = rot_quat.to_axis_angle()
# angle = np.rad2deg(angle)
# print("Angle: {} deg".format(angle))
# print("Axis:  {}".format(axis))
# print("Prod:  {}".format(angle * axis))
# print()
print("Pitch error: {} deg".format(rot_euler[0]))
print("Roll error: {} deg".format(rot_euler[1]))
print("Yaw error: {} deg".format(rot_euler[2]))
print()

# qv = np.array([rot_quat.x, rot_quat.y, rot_quat.z], dtype=np.float32)
# qv_mag = np.linalg.norm(qv)
# qr = rot_quat.w
# e_mag = 2 * np.arctan(qv_mag / qr)
# e = e_mag * qv
# print("e_mag: {}".format(e_mag))
# print("e: {}".format(e))
