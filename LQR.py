import numpy as np

from plant import DroneConfig, DronePlant, g, b2w_rotatation
import matplotlib.pyplot as plt

"""
LQR.py
======

Linear-Quadratic Regulator for the X-configured drone defined in plant.py.

Pipeline:
    1. Pick an equilibrium (hover: level attitude, zero velocity, thrust = weight).
    2. Linearize the nonlinear dynamics about that equilibrium (finite differences)
       to get  dx_dot = A * dx + B * du.
    3. Solve the continuous-time algebraic Riccati equation (CARE)

           A^T P + P A - P B R^-1 B^T P + Q = 0

       "by hand", via the Hamiltonian-matrix / eigendecomposition method, instead
       of calling scipy.linalg.solve_continuous_are.
    4. Build the optimal gain  K = R^-1 B^T P  and use  u = u_eq - K (x - x_eq).

State vector (matches plant.py):
    x = [x, y, z, vx, vy, vz, phi, theta, psi, wx, wy, wz]           (12,)

Control vector used for linearization / LQR (the "virtual" actuators):
    u = [thrust, tau_roll, tau_pitch, tau_yaw]                        (4,)

"""

N_STATES = 12
N_INPUTS = 4



def nonlinear_dynamics(state, u, config):
    x, y, z, vx, vy, vz, phi, theta, psi, wx, wy, wz = state
    thrust, tau_x, tau_y, tau_z = u
    m = config.mass

    RM_b2w = b2w_rotatation(phi, theta, psi)
    thrust_world = RM_b2w @ np.array([0.0, 0.0, -thrust])
    gravity = np.array([0.0, 0.0, g])

    dvx, dvy, dvz = (1.0 / m) * thrust_world + gravity

    euler_angle_matrix = np.array([
        [1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi) / np.cos(theta), np.cos(phi) / np.cos(theta)],
    ])
    dphi, dtheta, dpsi = euler_angle_matrix @ np.array([wx, wy, wz])

    Ixx = config.inertia[0, 0]
    Iyy = config.inertia[1, 1]
    Izz = config.inertia[2, 2]

    dwx = ((Iyy - Izz) * wy * wz) / Ixx + tau_x / Ixx
    dwy = ((Izz - Ixx) * wx * wz) / Iyy + tau_y / Iyy
    dwz = ((Ixx - Iyy) * wx * wy) / Izz + tau_z / Izz

    return np.array([vx, vy, vz, dvx, dvy, dvz, dphi, dtheta, dpsi, dwx, dwy, dwz])


def hover_equilibrium(config):
    """Level attitude, zero velocity/rate, thrust balances weight."""
    x_eq = np.zeros(N_STATES)
    u_eq = np.array([config.mass * g, 0.0, 0.0, 0.0])
    return x_eq, u_eq


def linearize(config, x_eq, u_eq, eps=1e-6):
    """
    Central-difference Jacobians of nonlinear_dynamics about (x_eq, u_eq):
        A = df/dx,   B = df/du
    """
    n, m = N_STATES, N_INPUTS
    A = np.zeros((n, n))
    B = np.zeros((n, m))

    for i in range(n):
        dx = np.zeros(n)
        dx[i] = eps
        f_plus = nonlinear_dynamics(x_eq + dx, u_eq, config)
        f_minus = nonlinear_dynamics(x_eq - dx, u_eq, config)
        A[:, i] = (f_plus - f_minus) / (2 * eps)

    for i in range(m):
        du = np.zeros(m)
        du[i] = eps
        f_plus = nonlinear_dynamics(x_eq, u_eq + du, config)
        f_minus = nonlinear_dynamics(x_eq, u_eq - du, config)
        B[:, i] = (f_plus - f_minus) / (2 * eps)

    return A, B



def solve_care(A, B, Q, R):
    n = A.shape[0]
    R_inv = np.linalg.inv(R)

    H = np.block([
        [A, -B @ R_inv @ B.T],
        [-Q, -A.T],
    ])

    eigvals, eigvecs = np.linalg.eig(H)

    # indices of the n eigenvalues with the most negative real part
    stable_idx = np.argsort(eigvals.real)[:n]
    stable_vals = eigvals[stable_idx]

    if np.any(stable_vals.real >= -1e-9):
        raise ValueError(
            "Hamiltonian doesn't have n eigenvalues with negative real part "
            "-- check that (A, B) is stabilizable and (A, Q^{1/2}) is detectable."
        )

    V = eigvecs[:, stable_idx]
    X1 = V[:n, :]
    X2 = V[n:, :]

    P = X2 @ np.linalg.inv(X1)

    P = (P + P.conj().T) / 2
    return np.real(P)


class LQRController:
    """
    LQR controller linearized about hover for the DronePlant defined in
    plant.py.

    Usage:
        controller = LQRController(config)
        u = controller.control(state)                # [thrust, tau_r, tau_p, tau_y]
        omega = controller.control_motor_speeds(state)  # 4 motor speeds
    """

    def __init__(self, config: DroneConfig, Q=None, R=None):
        self.config = config

        self.x_eq, self.u_eq = hover_equilibrium(config)
        self.A, self.B = linearize(config, self.x_eq, self.u_eq)

        if Q is None:
            # state order: [x,y,z, vx,vy,vz, phi,theta,psi, wx,wy,wz]
            Q = np.diag([10, 10, 10,
                         1, 1, 1,
                         50, 50, 20,
                         1, 1, 1]).astype(float)
        if R is None:
            # input order: [thrust, tau_roll, tau_pitch, tau_yaw]
            R = np.diag([0.5, 20, 20, 20]).astype(float)

        self.Q = Q
        self.R = R

        self.P = solve_care(self.A, self.B, self.Q, self.R)
        self.K = np.linalg.inv(self.R) @ self.B.T @ self.P

        # A throwaway plant instance
        self.mixer = DronePlant(config, np.zeros(N_STATES))

    def closed_loop_eigenvalues(self):
        """Eigenvalues of A - B K; all should have negative real part."""
        return np.linalg.eigvals(self.A - self.B @ self.K)

    def control(self, state, ref=None):
        """
        state: length-12 array, current [x,y,z, vx,vy,vz, phi,theta,psi, wx,wy,wz]
        ref:   optional length-12 setpoint (defaults to hover at the origin)

        Returns u = [thrust, tau_roll, tau_pitch, tau_yaw]
        """
        if ref is None:
            ref = self.x_eq

        error = np.asarray(state, dtype=float) - np.asarray(ref, dtype=float)

        # wrap yaw error into [-pi, pi] so the controller doesn't fight the
        # 2*pi wraparound ambiguity
        error[8] = np.arctan2(np.sin(error[8]), np.cos(error[8]))

        u = self.u_eq - self.K @ error
        u[0] = max(u[0], 0.0)  # thrust can't be negative
        return u

    def control_motor_speeds(self, state, ref=None):
        """Same as control(), but converted to the 4 motor speeds via inverse_mixing."""
        u = self.control(state, ref)
        thrust, torques = u[0], u[1:4]
        return self.mixer.inverse_mixing(thrust, torques)



if __name__ == "__main__":
    config = DroneConfig(
        mass=1.2,
        inertia=np.diag([0.02, 0.02, 0.04]),
        length=0.25,
        kd=0.01,
        kt=3e-5,
        kb=1e-6,
    )

    controller = LQRController(config)

    print("Closed-loop eigenvalues (should all have negative real part):")
    print(np.round(controller.closed_loop_eigenvalues(), 3))

    # Start tipped over and off-center, with some initial spin.
    x0 = np.zeros(N_STATES)
    x0[0:3] = [1.0, -0.5, 2.0]       # position offset
    x0[6:9] = [0.3, -0.2, 0.5]       # roll, pitch, yaw offset (rad)
    x0[9:12] = [0.1, -0.1, 0.05]     # initial body rates (rad/s)

    plant = DronePlant(config, x0.copy())

    dt = 0.01
    steps = 2000  # 20 seconds
    history = np.zeros((steps + 1, N_STATES))
    history[0] = x0

    state = x0.copy()
    for k in range(steps):
        u = controller.control(state)
        thrust, torques = u[0], u[1:4]
        state = plant.step(thrust, torques, dt)
        history[k + 1] = state

    print("\nFinal state [x,y,z, vx,vy,vz, phi,theta,psi, wx,wy,wz]:")
    print(np.round(history[-1], 4))

    t = np.arange(steps + 1) * dt
    fig, axes = plt.subplots(2, 2, figsize=(10, 7))

    axes[0, 0].plot(t, history[:, 0:3])
    axes[0, 0].set_title("Position")
    axes[0, 0].legend(["x", "y", "z"])

    axes[0, 1].plot(t, history[:, 6:9])
    axes[0, 1].set_title("Attitude (rad)")
    axes[0, 1].legend(["phi", "theta", "psi"])

    axes[1, 0].plot(t, history[:, 3:6])
    axes[1, 0].set_title("Velocity")
    axes[1, 0].legend(["vx", "vy", "vz"])

    axes[1, 1].plot(t, history[:, 9:12])
    axes[1, 1].set_title("Body rates (rad/s)")
    axes[1, 1].legend(["wx", "wy", "wz"])

    for ax in axes.flat:
        ax.set_xlabel("time (s)")
        ax.grid(True)

    fig.tight_layout()
    fig.savefig("lqr_response.png", dpi=150)
    print("\nSaved response plot to lqr_response.png")
