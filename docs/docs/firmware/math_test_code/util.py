
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
    
    ## Convert to euler angles: pitch (about x), roll (about y), and yaw (about z)
    #  EULER ANGLES USE EXTRINSIC ROTATIONS!!!
    #  First around world X, then around world Y, then around world Z
    def to_pry(self) -> Tuple[np.float32, np.float32, np.float32]:
        t2 = +2.0 * (self.w * self.y - self.z * self.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        roll = np.arcsin(t2)
        pitch = np.float32(0.0)
        yaw = np.float32(0.0)
        if np.abs(90.0 - np.abs(180.0 * roll / np.pi)) < 0.1:
            # Roll is +/- 90 degrees
            # Pitch and yaw mean the same thing (gimbal lock)
            # pitch + yaw = 2 * atan(self.x, self.w)
            # Can split any way between pitch and yaw
            # Choose to put it all in pitch
            pitch = 2.0 * np.arctan2(self.x, self.w)
            yaw = 0.0
        else:
            t0 = +2.0 * (self.w * self.x + self.y * self.z)
            t1 = +1.0 - 2.0 * (self.x * self.x + self.y * self.y)
            pitch = np.arctan2(t0, t1)
            t3 = +2.0 * (self.w * self.z + self.x * self.y)
            t4 = +1.0 - 2.0 * (self.y * self.y + self.z * self.z)
            yaw = np.arctan2(t3, t4)
        return [pitch * 180.0 / np.pi, roll * 180.0 / np.pi, yaw * 180.0 / np.pi]

    ## Construct quaternion from pitch (about x), roll (about y), and yaw (about z)
    #  EULER ANGLES USE EXTRINSIC ROTATIONS!!!
    #  First around world X, then around world Y, then around world Z
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
        if q.w > 1:
            q = self.normalize()
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

    def norm2(self) -> np.float32:
        return np.sqrt(self.w*self.w + self.x*self.x + self.y*self.y + self.z*self.z)

    def normalize(self) -> 'Quaternion':
        return self.divide_scalar(self.norm2())

