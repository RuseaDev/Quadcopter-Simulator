import numpy as np
from dataclasses import dataclass

g = 9.81 # gravitational acceleration

"""
Our drone will be an X-configured drone.
Motor 1      Motor 2
          |
          | 
          |
Motor 4      Motor 3

Motor 1 and 3 spins clockwise.
Motor 2 and 4 spin counterclockwise.

"""

@dataclass
class DroneConfig:
    mass: float
    inertia: np.ndarray
    length: float
    # coefficients for thrust and drag will come later
    kd: float
    kt: float # coefficient for tau_roll and tau_pitch
    kb: float # coefficient for tau_yaw

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
def RK4_step(f, t, y, h):
    k1 = f(t, y)
    k2 = f(t + h/2, y + (h/2) * k1)
    k3 = f(t + h/2, y + (h/2) * k2)
    k4 = f(t + h, y + h * k3)

    return y + (h / 6) * (k1 + 2*k2 + 2*k3 + k4)


class DronePlant:
    def __init__(self, config, state_vectors):
        self.config = config
        self.position = state_vectors[0:3]
        self.velocity = state_vectors[3:6]
        self.euler_angles = state_vectors[6:9]
        self.angular_velocity = state_vectors[9:12]

    # Return torques
    # def motor_mixing(self):
    #     # Torques and thrust will be written here
    #     # How would I really write the torques here? 
    #     # Should return omega?
    #     length = self.config.length
    #     kt = self.config.kt
    #     kb = self.config.kb
    #     perpendicular_length = length/np.sqrt(2)

    #     tau_roll, tau_pitch, tau_yaw = self.torques
    #     tau_roll = kt*(-perpendicular_length * omega_1 ** 2 - perpendicular_length * omega_2 ** 2 + perpendicular_length + omega_3 ** 2 + perpendicular_length + omega_4 ** 2)
    #     tau_pitch = -perpendicular_length * omega_1 ** 2 + perpendicular_length * omega_2 ** 2 - perpendicular_length + omega_3 ** 2 + perpendicular_length + omega_4 ** 2
    #     tau_yaw = kb( -omega_1**2 + omega_2 ** 2 - omega_3 ** 2 + omega_4 ** 2)

        # return tau_roll, tau_pitch, tau_yaw
    
    def state_derivatives(self, thrust, torques):

        m = self.config.mass # mass of the drone
 

        x, y, z = self.position
        vx, vy, vz = self.velocity
        phi, theta, psi = self.euler_angles
        wx, wy, wz = self.angular_velocity

        RM_b2w = b2w_rotatation(phi, theta, psi) # Rotation to convert coordinates from body to inertial frame

        thrust_vector = np.array([0, 0, -thrust]).T
        
        thrust_world = RM_b2w @ thrust_vector 
        gravity = np.array([0, 0, g]).T
        
        # Instead of saying x_dot, y_dot, vx_dot, etc..., we will use dx, dy, ...

        dx = vx
        dy = vy
        dz = vz

        dvelocity = (1.0 / m) * (thrust_world) + gravity
        dvx, dvy, dvz = dvelocity

        # Rotational dynamics

        euler_angle_matrix = np.array([[1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
                                       [0, np.cos(phi), -np.sin(phi)],
                                       [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])

        Ixx = self.config.inertia[0,0]
        Iyy = self.config.inertia[1,1]
        Izz = self.config.inertia[2,2]

        tau_x, tau_y, tau_z = torques


        dphi, dtheta, dpsi = euler_angle_matrix @ np.array([wx, wy, wz])

        dwx = ((Iyy - Izz) * wy * wz) / Ixx + tau_x / Ixx
        dwy = ((Izz - Ixx) * wx * wz) / Iyy + tau_y / Iyy
        dwz = ((Ixx - Iyy) * wx * wy) / Izz + tau_z / Izz

        return np.array([dx, dy, dz, 
                         dvx, dvy, dvz, 
                         dphi, dtheta, dpsi, 
                         dwx, dwy, dwz])


dt = 0.01

t = np.arange(0, 10, dt)

print(t)
# dt = 0.01

# t = np.arange(0, 10, dt)

# print(t)







