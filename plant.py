import numpy as np
from dataclasses import dataclass

g = 9.81 # gravitational acceleration

@dataclass
class DroneConfig:
    mass: float
    inertia: np.ndarray
    length: float
    # coefficients for thrust and drag will come later

"""
Rotational Matrix from body to world frame:
R(phi)@R(theta)@R(psi) aka. R(roll)@R(pitch)@R(yaw)
"""
def r_roll(phi):
    return np.array([[1, 0, 0],
                     [0, np.cos(phi), -np.sin(phi)],
                     [0, np.sin(phi), np.cos(phi)]])

def r_pitch(theta):
    return np.array([[np.cos(theta), 0, np.sin(theta)],
                     [0, 1, 0],
                     [-np.sin(theta), 0, np.cos(theta)]])

def r_yaw(psi):
    return np.array([[np.cos(psi), -np.sin(psi), 0],
                     [np.sin(psi), np.cos(psi), 0],
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
    def __init__(self, config, state_vectors, torques):
        self.config = config
        self.position = state_vectors[0:3]
        self.velocity = state_vectors[3:6]
        self.euler_angles = state_vectors[6:9]
        self.omega = state_vectors[9:12]
        self.torques = torques


    def translational_dynamics(self, thrust):
        '''
        Function's Purpose: Derivatives for x, y, z, v_x, v_y, v_z        
        Input: 
            - Force of Body Frame: Thrust, Drag, Gravity
            - Sensor's Velocity 
            - Drone's mass
            - Euler's Angles
            - g = 9.81
        
        Calculate Velocity Derivative: 
            - Change force of body frame to intertial frame => Feed Euler angles to b2w rotational matrix 
            
            - Calculate d(Velocity) using the formula:
                Ftotal = thrust + drag + grav
                => m * a = thrust + drag + grav
                => a = (1 / m) * (thrust + drag + m * g)
                => v_dot = (1 / m) * (thrust + drag) + g

        Calculate the Positional Derivative: 
            - Update the current velocity using acceleration over a very small time step - t = 0.001

        '''
        
        '''
        Note for the future: 
            - This code temporarily don't have drag because it has not been calculated 
            - Future code will have to include drag
        '''
        # Grabbing input
        m = self.config.mass
        phi, theta, psi = self.euler_angles
        RM_b2w = b2w_rotatation(phi, theta, psi)

        thrust_world = RM_b2w @ thrust 
        # drag_world = RM_b2w @ drag
        gravity = np.matrix([0, 0, g]).T
        delta_time = 0.001

        # Calculate velocity derivative
        # velocity_dot = (1.0 / m) * (thrust_world + drag_world) + gravity
        
        velocity_dot = (1.0 / m) * thrust_world + gravity
        position_dot = self.velocity 
    
        return velocity_dot, position_dot

                   
    def rotational_dynamics(self):
        # Derivatives for roll, pitch, yaw, omega_x, omega_y, omega_z 
        w_x, w_y, w_z = self.omega
        phi, theta, psi = self.euler_angles

        euler_angle_matrix = np.array([[1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
                                       [0, np.cos(phi), -np.sin(phi)],
                                       [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])
        
        Ixx = self.config.inertia[0,0]
        Iyy = self.config.inertia[1,1]
        Izz = self.config.inertia[2,2]

        tau_x, tau_y, tau_z = self.torques
        
        phi_dot, theta_dot, psi_dot = euler_angle_matrix @ np.array([w_x, w_y, w_z])
        w_x_dot = ((Iyy - Izz) * w_y * w_z) / Ixx + tau_x / Ixx
        w_y_dot = ((Izz - Ixx) * w_x * w_z) / Iyy + tau_y / Iyy
        w_z_dot = ((Ixx - Iyy) * w_x * w_y) / Izz + tau_z / Izz

        return phi_dot, theta_dot, psi_dot, w_x_dot, w_y_dot, w_z_dot
        


    def motor_mixing(self):
        # Torques and thrust will be written here
        # How would I really write the torques here? 
    