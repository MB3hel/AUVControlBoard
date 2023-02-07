
import numpy as np
from util import Quaternion


# To quat then back to euler "simplifies" the angles
# pitch from -180 to 180
# roll from -90 to 90
# yaw from -180 to 180 (note the bno055 uses 0 to 360, but quaternions will be the same)
prev_quat = Quaternion.from_pry(0.0, 0.0, 0.0)
curr_quat = Quaternion.from_pry(0.0, 90.0, 0.0)

print("Prev Quat: {}".format(prev_quat))
print("Curr Quat: {}".format(curr_quat))
print()

# Calculate smallest explication of angle change between the two given quaternions
diff_quat = Quaternion()
if curr_quat.dot(prev_quat) < 0:
    diff_quat = curr_quat.multiply(prev_quat.multiply_scalar(-1).inverse())
else:
    diff_quat = curr_quat.multiply(prev_quat.inverse())
diff_euler = diff_quat.to_pry()

print("Diff Quat: {}".format(diff_quat))
print("Diff Euler: {}".format(diff_euler))

# Note: The values in the diff euler would be added to accumulated pitch, roll, and yaw
