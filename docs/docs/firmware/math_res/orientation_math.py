
import numpy as np
from typing import Tuple


class Quaternion:
    def __init__(self, w: np.float32 = 0.0, x: np.float32 = 0.0, y: np.float32 = 0.0, z: np.float32 = 0.0):
        self.w: np.float32 = np.float32(w)
        self.x: np.float32 = np.float32(x)
        self.y: np.float32 = np.float32(y)
        self.z: np.float32 = np.float32(z)
    
    def __repr__(self) -> str:
        return "Quaternion(w = {}, x = {}, y = {}, z = {})".format(self.w, self.x, self.y, self.z)
    
    def to_pry(self) -> Tuple[np.float32, np.float32, np.float32]:
        t0 = +2.0 * (self.w * self.x + self.y * self.z)
        t1 = +1.0 - 2.0 * (self.x * self.x + self.y * self.y)
        pitch = np.arctan2(t0, t1)
        t2 = +2.0 * (self.w * self.y - self.z * self.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        roll = np.arcsin(t2)
        t3 = +2.0 * (self.w * self.z + self.x * self.y)
        t4 = +1.0 - 2.0 * (self.y * self.y + self.z * self.z)
        yaw = np.arctan2(t3, t4)
        return [pitch * 180.0 / np.pi, roll * 180.0 / np.pi, yaw * 180.0 / np.pi]

    @staticmethod
    def from_pry(p: np.float32, r: np.float32, y: np.float32) -> 'Quaternion':
        p = np.float32(p * np.pi / 180.0)
        r = np.float32(r * np.pi / 180.0)
        y = np.float32(y * np.pi / 180.0)
        qx = np.sin(p/2) * np.cos(r/2) * np.cos(y/2) - np.cos(p/2) * np.sin(r/2) * np.sin(y/2)
        qy = np.cos(p/2) * np.sin(r/2) * np.cos(y/2) + np.sin(p/2) * np.cos(r/2) * np.sin(y/2)
        qz = np.cos(p/2) * np.cos(r/2) * np.sin(y/2) - np.sin(p/2) * np.sin(r/2) * np.cos(y/2)
        qw = np.cos(p/2) * np.cos(r/2) * np.cos(y/2) + np.sin(p/2) * np.sin(r/2) * np.sin(y/2)
        return Quaternion(qw, qx, qy, qz)

    def to_axis_angle(self) -> Tuple[np.float32, np.array]:
        q = self
        # TODO: Normalize q if needed (q.w > 1 need to normalize)
        angle = np.float32(2) * np.arccos(q.w)
        s = np.sqrt(1 - q.w * q.w)
        axis = np.array([0, 0, 0], dtype=np.float32)
        if s == np.float32(0) or s == np.float32(-0.0):
            axis[0] = np.float32(1)
            axis[1] = np.float32(0)
            axis[2] = np.float32(0)
        else:
            axis[0] = q.x / s
            axis[1] = q.y / s
            axis[2] = q.z / s
        return angle, axis


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

    def norm(self) -> np.float32:
        return np.sqrt(self.w*self.w + self.x*self.x + self.y*self.y + self.z*self.z)



################################################################################
# Convert euler to quaternion
################################################################################

curr_quat = Quaternion.from_pry(0, 90, 0)
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
