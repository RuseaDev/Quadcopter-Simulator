import numpy as np
from plant import DroneConfig, DronePlant, g, b2w_rotatation

"""
LQR.py

Linear-Quadratic Regular for the X-configured drone defined in plant.py

Pipeline:
1. Pick an equilibrium (hover: level attitude, zero velocity, thrust = weight)

2. Linearize the nonlinear dynamics about the equilibrium piont

3. Solve the continuous-time algebraic Riccati equation (CARE)

A^TP + PA - PBR^-1B^TP + Q = 0

4. Build the optimal gain K = R^-1B^TP and use u = u_eq - K(x - x_eq)


State vector (matches plant.py)
x = [x, y, z, vx, vy, vz, phi, theta, psi, wx, wy, wz]

Control vector used for linearization (the virtual actuators):
u = [thrust, tau_roll, tau_pitch, tau_yaw]
"""

N_STATES = 12
N_INPUTS = 4


def nonlinear_dynamics(state, u, config):
    # Returns a vector of the derivatives of the state


def hover_equilibrium(config):
    # Level attitude, zero velocity/rate, thrust balances weight

    x_eq  = np.zeros(N_STATES)
    u_eq = np.array([config.mass * g, 0.0, 0.0, 0.0])

    return x_eq, u_eq


def linearize(config, x_eq, u_eq, eps=1e-6):
    # Central-difference Jacobians of nonlinear-dynamics about (x_eq, u_eq)

    # A = df/dx, B = df/du


    return A, B

class LQRController:
    def __init__(self, config: DroneConfig, Q, R):
        self.config = config
        self.x_eq = hover_equilibrium(config)
        