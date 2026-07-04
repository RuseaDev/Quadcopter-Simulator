"""
Simulates the cascaded PID controller (pid.py) flying the quadrotor
plant (plant.py) through a waypoint sequence, then plots the results:

  1. 3D flight path
  2. Position (x, y, altitude) vs. time, with commanded setpoints
  3. Attitude (roll, pitch, yaw) vs. time, with the outer loop's
     commanded attitude
  4. Thrust and body torques vs. time
  5. Motor speeds (rad/s) implied by the commanded thrust/torques

Run with:  python3 visualization.py
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3D projection)

from plant import DroneConfig, DronePlant
from pid import CascadedController


# ----------------------------------------------------------------------
# Setup
# ----------------------------------------------------------------------
def make_config():
    return DroneConfig(
        mass=1.0,                                   # kg
        inertia=np.diag([0.0123, 0.0123, 0.0224]),  # kg*m^2 (typical small quad)
        length=0.25,                                 # m, motor arm length
        kd=0.0,
        kt=3.13e-5,   # thrust coefficient
        kb=7.5e-7,    # yaw drag/torque coefficient
    )


def waypoint_schedule(t):
    """
    Returns (pos_des, psi_des) for a given simulation time. The drone:
      1. Takes off straight up to 5 m
      2. Flies out to (4, 3) while yawing to 45 deg
      3. Returns toward the origin while yawing back to 0
    """
    if t < 4.0:
        return np.array([0.0, 0.0, 5.0]), 0.0
    elif t < 10.0:
        return np.array([4.0, 3.0, 5.0]), np.radians(45)
    else:
        return np.array([0.0, 0.0, 3.0]), 0.0


# ----------------------------------------------------------------------
# Simulation
# ----------------------------------------------------------------------
def run_simulation(t_final=16.0, dt=0.005):
    config = make_config()
    state0 = np.zeros(12)  # start on the ground, level, at rest
    plant = DronePlant(config, state0)
    controller = CascadedController(config, outer_rate_hz=50.0, attitude_rate_hz=250.0)

    n_steps = int(t_final / dt)
    log = {
        "t": np.zeros(n_steps),
        "pos": np.zeros((n_steps, 3)),
        "pos_des": np.zeros((n_steps, 3)),
        "euler": np.zeros((n_steps, 3)),
        "att_des": np.zeros((n_steps, 3)),
        "thrust": np.zeros(n_steps),
        "torques": np.zeros((n_steps, 3)),
        "omega": np.zeros((n_steps, 4)),
    }

    t = 0.0
    for i in range(n_steps):
        pos_des, psi_des = waypoint_schedule(t)

        thrust, torques = controller.update(pos_des, psi_des, plant, dt)
        motor_speeds = plant.inverse_mixing(thrust, torques)

        log["t"][i] = t
        log["pos"][i] = [plant.position[0], plant.position[1], -plant.position[2]]  # altitude = -z
        log["pos_des"][i] = pos_des
        log["euler"][i] = np.degrees(plant.euler_angles)
        log["att_des"][i] = np.degrees(controller._att_des)
        log["thrust"][i] = thrust
        log["torques"][i] = torques
        log["omega"][i] = motor_speeds

        plant.step(thrust, torques, dt)
        t += dt

    return log


# ----------------------------------------------------------------------
# Plotting
# ----------------------------------------------------------------------
def plot_results(log, save_path="drone_simulation.png"):
    t = log["t"]
    fig = plt.figure(figsize=(14, 10))

    # --- 3D trajectory ---------------------------------------------------
    ax3d = fig.add_subplot(2, 3, 1, projection="3d")
    ax3d.plot(log["pos"][:, 0], log["pos"][:, 1], log["pos"][:, 2], color="tab:blue", label="flown path")
    ax3d.scatter(*log["pos"][0, :], color="green", label="start")
    ax3d.scatter(*log["pos"][-1, :], color="red", label="end")
    ax3d.set_xlabel("x (m)")
    ax3d.set_ylabel("y (m)")
    ax3d.set_zlabel("altitude (m)")
    ax3d.set_title("3D flight path")
    ax3d.legend(fontsize=8)

    # --- position vs time --------------------------------------------------
    ax_pos = fig.add_subplot(2, 3, 2)
    labels = ["x", "y", "altitude"]
    colors = ["tab:blue", "tab:orange", "tab:green"]
    for i, (label, c) in enumerate(zip(labels, colors)):
        ax_pos.plot(t, log["pos"][:, i], color=c, label=f"{label} actual")
        ax_pos.plot(t, log["pos_des"][:, i], color=c, linestyle="--", alpha=0.6, label=f"{label} desired")
    ax_pos.set_xlabel("time (s)")
    ax_pos.set_ylabel("position (m)")
    ax_pos.set_title("Position tracking")
    ax_pos.legend(fontsize=7, ncol=2)
    ax_pos.grid(alpha=0.3)

    # --- attitude vs time --------------------------------------------------
    ax_att = fig.add_subplot(2, 3, 3)
    att_labels = ["roll (phi)", "pitch (theta)", "yaw (psi)"]
    for i, (label, c) in enumerate(zip(att_labels, colors)):
        ax_att.plot(t, log["euler"][:, i], color=c, label=f"{label} actual")
        ax_att.plot(t, log["att_des"][:, i], color=c, linestyle="--", alpha=0.6, label=f"{label} desired")
    ax_att.set_xlabel("time (s)")
    ax_att.set_ylabel("angle (deg)")
    ax_att.set_title("Attitude tracking")
    ax_att.legend(fontsize=7, ncol=2)
    ax_att.grid(alpha=0.3)

    # --- thrust vs time ------------------------------------------------
    ax_thrust = fig.add_subplot(2, 3, 4)
    ax_thrust.plot(t, log["thrust"], color="tab:purple")
    ax_thrust.set_xlabel("time (s)")
    ax_thrust.set_ylabel("thrust (N)")
    ax_thrust.set_title("Commanded thrust")
    ax_thrust.grid(alpha=0.3)

    # --- torques vs time -----------------------------------------------
    ax_torque = fig.add_subplot(2, 3, 5)
    torque_labels = ["tau_x (roll)", "tau_y (pitch)", "tau_z (yaw)"]
    for i, (label, c) in enumerate(zip(torque_labels, colors)):
        ax_torque.plot(t, log["torques"][:, i], color=c, label=label)
    ax_torque.set_xlabel("time (s)")
    ax_torque.set_ylabel("torque (N*m)")
    ax_torque.set_title("Commanded torques")
    ax_torque.legend(fontsize=7)
    ax_torque.grid(alpha=0.3)

    # --- motor speeds vs time -------------------------------------------
    ax_motor = fig.add_subplot(2, 3, 6)
    for i in range(4):
        ax_motor.plot(t, log["omega"][:, i], label=f"motor {i+1}")
    ax_motor.set_xlabel("time (s)")
    ax_motor.set_ylabel("omega (rad/s)")
    ax_motor.set_title("Motor speeds (inverse_mixing)")
    ax_motor.legend(fontsize=7)
    ax_motor.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    print(f"Saved plot to {save_path}")
    plt.show()


if __name__ == "__main__":
    sim_log = run_simulation()
    plot_results(sim_log)