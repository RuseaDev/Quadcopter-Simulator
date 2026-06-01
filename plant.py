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
        self.position = state_vectors[0:3]
        self.velocity = state_vectors[3:6] 
        self.euler_angles = state_vectors[6:9] # phi, theta, psi
        self.omega = state_vectors[9:12] # body frame angular velocity

    def derivatives(self):
        '''
        Purpose: Calculate the Euler rates, knowing: 
            - Body frame Angulr Velocity: self.omega
            - Euler Angles: self.euler_angles

        Euler rates:
            - psi_dot = (w_y * sin(phi) + w_z * cos(phi)) * sec(theta)
            - theta_dot = w_y * cos(phi) - w_z * sin (phi)
            - phi_dot = w_x + (w_y * sin(phi) + w_z * cos (phi)) * tan(theta)
        '''
        
        #Input
        wx, wy, wz = self.omega
        phi, theta, psi = self.euler_angles
        sin_phi = np.sin (phi)
        cos_phi = np.cos (phi) 
        sec_theta = 1.0 / np.cos(theta)
        tan_theta = np.tan (theta)
        
        #Calculate Euler rates
        psi_dot = (wy * sin_phi + wz * cos_phi) * sec_theta
        theta_dot = wy * cos_phi - wz * sin_phi
        phi_dot = wx + (wy * sin_phi + wz * cos_phi) * tan_theta
        
        return phi_dot, theta_dot, psi_dot

    def translational_dynamics(self):
        # What should this be for exactly? 
        # How does this function differ from derivatives function?
        return 

    def rotational_dynamics(self):
        # What about this? 
        return 

    def motor_mixing(self):
        #
        return 