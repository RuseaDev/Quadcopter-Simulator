"""
pid.visualizer.py

Runs a closed-loop simulation of the cascaded PID drone
controller against the nonlinear rigid-body plant
"""


import numpy as np
import matplotlib.pyplot as plt

from plant import DroneConfig, DronePlant
from pid_controller import CascadedController


DT = 0.0002 # rate-loop timestep(s). Frequency = 1/0.002 = 500 Hz
BASE_ALTITUDE = 1.5 # m


"""
Waypoints: (time, [x, y, altitude], yaw)
"""

WAYPOINTS = [
    (0.0,  np.array([0.0, 0.0, BASE_ALTITUDE]), 0.0),
    (2.0,  np.array([0.0, 0.0, BASE_ALTITUDE]), 0.0),
    (8.0,  np.array([3.0, 2.0, 2.2]),           np.radians(45)),
    (11.0, np.array([3.0, 2.0, 2.2]),           np.radians(45)),
    (17.0, np.array([0.0, 0.0, BASE_ALTITUDE]), 0.0),
    (20.0, np.array([0.0, 0.0, BASE_ALTITUDE]), 0.0),
]
SIM_TIME = WAYPOINTS[-1][0] # End time of sim


def make_drone_config():
    return DroneConfig(
        mass = 1.0, # kg
        inertia = np.diag([0.01, 0.01, 0.02]), # kg*m^2
        length = 0.25, # m, motor arm length

        kd = 0.0, # drag coefficient

        kt = 3.0e-5, # thrust coefficient
        kb = 1.0e-6 # yaw-drag coefficient
    )

def minjerk(tau):
    tau = np.clip(tau, 0.0, 1.0)
    return 10 * tau**3 - 15 * tau**4 + 6 * tau**5


def desired_trajectory(t):
    """ 
    Returns (pos_des, psi_des) at time t by interpolating WAYPOINTS
    """
    if t <= WAYPOINTS[0][0]:
        return WAYPOINTS[0][1], WAYPOINTS[0][2]
    
    for (t0, p0, y0), (t1, p1, y1) in zip(WAYPOINTS[:-1], WAYPOINTS[1:]):
        if t0 <= t <= t1:
            tau = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
            s = minjerk(tau)
            return p0 + (p1-p0) * s, y0 + (y1-y0) * s
    
    return WAYPOINTS[-1][1], WAYPOINTS[-1][2]
            
        
    # to be continued


def run_simulation():
    config = make_drone_config()

    state0 = np.zeros(12) # Initial staet at rest hovering at the start altitude
    state0[2] = -BASE_ALTITUDE

    plant = DronePlant(config, state0)

    controller = CascadedController(config, outer_rate_hz = 50, attitude_rate_hz=5000)

    n_steps = int (SIM_TIME / DT)

    log = {
        "t": np.zeros(n_steps),
        "pos": np.zeros((n_steps, 3)), # actual x, y, altitude
        "pos_des": np.zeros((n_steps, 3)) ,

        "att": np.zeros((n_steps, 3)),      # actual phi, theta, psi
        "att_des": np.zeros((n_steps, 3)),
        "rate": np.zeros((n_steps, 3)),     # body angular velocity
        "thrust": np.zeros(n_steps),
        "torques": np.zeros((n_steps, 3)),
    }

    for i in range(n_steps):
        t = i * DT
        pos_des, psi_des = desired_trajectory(t)

        thrust, torques = controller.update(pos_des, psi_des, plant, DT)
        plant.step(thrust, torques, DT)

        x, y, z = plant.position
        log["t"][i] = t
        log["pos"][i] = [x, y, -z]
        log["pos_des"][i] = pos_des
        log["att"][i] = plant.euler_angles
        log["att_des"][i] = controller.att_des
        log["rate"][i] = plant.angular_velocity
        log["thrust"][i] = thrust
        log["torques"][i] = torques

    return log

def plot_trajectory_3d(log):
    fig = plt.figure()

    ax = fig.add_subplot(111, projection="3d")

    ax.plot(log["pos"][:, 0], log["pos"][:, 1], log["pos"][:, 2], label="actual", color="blue")
    ax.plot(log["pos_des"][:, 0], log["pos_des"][:, 1], log["pos_des"][:, 2], label="desired", color="red")

    ax.scatter(*log["pos"][0], color="green", s=40, label="start")

    ax.set_xlabel(" x[m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("altitude [m]")  
    ax.set_title("3D Trajectory")
    ax.legend()
    fig.tight_layout()
    return fig


def main():
    log = run_simulation()

    fig_3d_trajectory = [(plot_trajectory_3d(log), "trajectory_3d.png")]

    for fig, name in fig_3d_trajectory:
        fig.savefig(name, dpi=150)
        print(f"Saved {name}")

    plt.show()


if __name__ == "__main__":
    main()