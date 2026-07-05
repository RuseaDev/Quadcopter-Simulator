import math
import numpy as np

class Quaternion:
    def __init__(self, w, x, y, z):
        self.w, self.x, self.y, self.z = w, x, y, z
    
    def __mul__(self, other):
        w = self.w*other.w - self.x*other.x - self.y*other.y - self.z*other.z
        x = self.w*other.x + self.x*other.w + self.y*other.z - self.z*other.y
        y = self.w*other.y - self.x*other.z + self.y*other.w + self.z*other.x
        z = self.w*other.z + self.x*other.y - self.y*other.x + self.z*other.w
        return Quaternion(w, x, y, z)
    
    def conjugate(self):
        return Quaternion(self.w, -self.x, -self.y, -self.z)
    
    def rotate(self, v):
        vx, vy, vz =  v
        vq = Quaternion(0, vx, vy, vz)
        q_res = self * vq * self.conjugate()
        return q_res
    
    def normalize(self):
        length = np.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)
        if length == 0:
            return Quaternion(self.w, self.x, self.y, self.z)
        return Quaternion(self.w / length, self.x / length, self.y / length, self.z / length)
    
    @classmethod
    def from_axis_angle(cls, axis, angle):
        angle_in_rad = np.deg2rad(angle/2)
        c = np.cos(angle_in_rad)
        s = np.sin(angle_in_rad)
        # Scalar part of quaternion:
        q_c = c
        # Vector part of quaternion:
        q_v = s * axis
        return Quaternion(q_c, *q_v)        
    
    def display(self):
        print(f"w: {self.w}, x: {self.x}, y: {self.y}, z: {self.z}")