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
def r_roll(phi):
    return np.array([[1, 0, 0],
                     [0, np.cos(phi), np.sin(phi)],
                     [0, -np.sin(phi), np.cos(phi)]])

def r_pitch(theta):
    return np.array([[np.cos(theta), 0, np.sin(theta)],
                     [0, 1, 0],
                     [-np.sin(theta), 0, np.cos(theta)]])

def r_yaw(psi):
    return np.array([[np.cos(psi), np.sin(psi), 0],
                     [-np.sin(psi), np.cos(psi), 0],
                     [0, 0, 1]])

def w2b_rotation(phi, theta, psi):

    return r_roll(phi) @ r_pitch(theta) @ r_yaw(psi)
    
def b2w_rotatation(phi, theta, psi):

    return r_yaw(psi) @ r_pitch(theta) @ r_roll(phi)
    

"""
RK4 integration method
"""
def RK4():
    return None


class DronePlant:
    def __init__(self, config, state_vectors):
        self.config = config
        self.position = state_vectors[0:3]
        self.velocity = state_vectors[3:6]
        self.euler_angles = state_vectors[6:9]
        self.omega = state_vectors[9:12]

    def translational_dynamics(self):
        # Derivatives for x, y, z, v_x, v_y, v_z

                   
    def rotational_dynamics(self):
        # Derivatives for roll, pitch, yaw, omega_x, omega_y, omega_z 
        


    def motor_mixing(self);
        #
    