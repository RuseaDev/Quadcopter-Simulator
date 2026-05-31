import numpy as np
from dataclasses import dataclass

g = 9.81 # gravitational acceleration

@dataclass
class DroneConfig:
    mass: float
    inertia: np.ndarray
    length: float
    # coefficients for thrust, drug will come later

"""
Rotational Matrix from body to world frame:
R(phi)@R(theta)@R(psi) aka. R(roll)@R(pitch)@R(yaw)
"""



class DronePlant:
    def __init__(self, config):

    def derivatives(self):

    def translational_dynamics(self):

    def rotational_dynamics(self):

    def motor_mixing(self);
    