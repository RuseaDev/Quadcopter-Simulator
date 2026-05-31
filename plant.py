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
def w2b_rotation(phi, theta, psi):

    #Rotation matrix for roll only
    r_roll = np.array([[1, 0 ,0],
                       [0, np.sin(phi), np.cos(phi)],
                       [0, np.sin(phi), np.sin(phi)]])
    
    #Rotat
    r_pitch = np.array([[1, 0 ,0],
                       [0, np.sin(phi), np.cos(phi)],
                       [0, np.sin(phi), np.sin(phi)]])
    r_yaw = np.array([[1, 0 ,0],
                       [0, np.sin(phi), np.cos(phi)],
                       [0, np.sin(phi), np.sin(phi)]])
    

"""
RK4 integration method
"""
def RK4():
    return None


class DronePlant:
    def __init__(self, config, state_vectors):
        self.config = config
        position = state_vectors[0:3]
        velocity = state_vectors[3:6]
        euler_angles = state_vectors[6:9]
        omega = state_vectors[9:12]

    def derivatives(self):
        # We have to find the formula for 

    def translational_dynamics(self):
        # What should this be for exactly? 
        # How does this function differ from derivatives function?

    def rotational_dynamics(self):
        # What about this? 

    def motor_mixing(self);
        #
    