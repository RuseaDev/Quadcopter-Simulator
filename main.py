import numpy as np
import matplotlib.pyplot as plt
from plant import DroneConfig, DronePlant
from mpu6050_imu_sim import (
    MPU6050Config,
    MPU6050Simulator,
    MPU6050_ACCEL_DEFAULT,
    MPU6050_GYRO_DEFAULT,
    plot_imu_data,
)

from madgwick_filter import MadgwickMARG
from pid_controller import CascadedController
from LQR import LQRController

def run_pipeline(duration_s=10.0, dt = 1.0 / 5000, seed = 0, controller_type='pid'):
    config = DroneConfig(
        mass=1.2,
        inertia=np.diag([0.02, 0.02, 0.04]),
        length=0.25,
        kd=0.01,
        kt=3e-5,
        kb=1e-6,
    )

    x0 = np.zeros(12)
    x0[0:3] = [1.0, -0.5, -2.0]     # NED: negative z = 2 m up
    x0[6:9] = [0.3, -0.2, 0.5]      # roll, pitch, yaw offset (rad)
    x0[9:12] = [0.1, -0.1, 0.05]    # initial body rates (rad/s)
    plant = DronePlant(config, x0.copy())

    if controller_type == "lqr":
        controller = LQRController(config)
    else:
        controller = CascadedController(config, outer_rate_hz=50, attitude_rate_hz=1.0/dt)

    pos_des = np.array([0.0, 0.0, 2.0])
    psi_des = 0.0

    n_steps = int(round(duration_s / dt))

    t_log = np.zeros(n_steps)
    pos_log = np.zeros((n_steps, 3))
    att_true_log = np.zeros((n_steps, 3))
    rate_true_log = np.zeros((n_steps, 3))
    accel_true_log = np.zeros((n_steps, 3))
    thrust_log = np.zeros(n_steps)
    torques_log = np.zeros((n_steps, 3))

    for i in range(n_steps):
        t_log[i] = i * dt

        if controller_type == "lqr":
            state = np.concatenate([plant.position, plant.velocity, plant.euler_angles, plant.angular_velocity])

            ref = np.zeros(12)
            ref[0:2] = pos_des[0:2]
            ref[2] = -pos_des[2]
            ref[8] = psi_des
            u = controller.control(state,ref)
            thrust, torques = u[0], u[1:4]

        else:
            thrust, torques = controller.update(pos_des, psi_des, plant, dt)

        plant.step(thrust, torques, dt)

        pos_log[i] = [plant.position[0], plant.position[1], -plant.position[2]]
        att_true_log[i] = plant.euler_angles
        rate_true_log[i] = plant.angular_velocity
        thrust_log[i] = thrust
        torques_log[i] = torques

        accel_true_log[i] = np.array([0.0, 0.0, thrust/config.mass])

    fs = 1.0 / dt

    imu_config = MPU6050Config(
        sample_rate_hz=fs,
        gyro_params=MPU6050_GYRO_DEFAULT,
        accel_params=MPU6050_ACCEL_DEFAULT,
        seed = seed,
    )

    imu = MPU6050Simulator(imu_config)

    _, gyro_meas, accel_meas = imu.simulate(rate_true_log, accel_true_log, duration_s
                                            )
    
    madgwick = MadgwickMARG(zeta=0.03, sample_rate_hz=fs)
    att_est_log = np.zeros((n_steps, 3))

    for i in range(n_steps):
        q = madgwick.update(gyro_meas[i], accel_meas[i], dt)
        att_est_log[i] = madgwick.quaternion_to_euler(q)

    return {
        "t": t_log,
        "pos": pos_log,
        "pos_des": pos_des,
        "att_true": att_true_log,
        "att_est": att_est_log,
        "rate_true": rate_true_log,
        "thrust": thrust_log,
        "torques": torques_log,
        "gyro_meas": gyro_meas,
        "accel_meas": accel_meas,
        "controller_type": controller_type,
    }

def plot_results(log):
    t = log["t"]

    fig1 = plt.figure(figsize=(7, 6))
    ax = fig1.add_subplot(projection="3d")
    ax.plot(log["pos"][:, 0], log["pos"][:, 1], log["pos"][:, 2], color="tab:blue", label="actual")
    ax.scatter(*log["pos"][0], color="green", s=40, label="start")
    ax.scatter(*log["pos_des"], color="red", s=40, label="setpoint")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("altitude [m]")
    ax.set_title(f"3D Trajectory ({log['controller_type']} controller)")
    ax.legend()
    fig1.tight_layout()

    fig2, axes2 = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    labels = ["x [m]", "y [m]", "altitude [m]"]
    for i in range(3):
        axes2[i].plot(t, log["pos"][:, i], label="actual")
        axes2[i].axhline(log["pos_des"][i], color="red", ls="--", label="setpoint")
        axes2[i].set_ylabel(labels[i])
        axes2[i].grid(alpha=0.3)
        axes2[i].legend(loc="upper right")
    axes2[-1].set_xlabel("time (s)")
    fig2.suptitle("Position Tracking")
    fig2.tight_layout()

    fig3, axes3 = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    att_labels = ["roll (phi)", "pitch (theta)", "yaw (psi)"]
    for i in range(3):
        axes3[i].plot(t, np.degrees(log["att_true"][:, i]), label="truth", color="tab:blue")
        axes3[i].plot(t, np.degrees(log["att_est"][:, i]), label="Madgwick estimate",
                      color="tab:orange", alpha=0.8)
        axes3[i].set_ylabel(f"{att_labels[i]} (deg)")
        axes3[i].grid(alpha=0.3)
        axes3[i].legend(loc="upper right")
    axes3[-1].set_xlabel("time (s)")
    fig3.suptitle("Attitude: Truth vs Madgwick Estimate")
    fig3.tight_layout()

    fig4, ax4 = plt.subplots(figsize=(9, 4))
    err_deg = np.degrees(log["att_est"] - log["att_true"])
    for i, name in enumerate(["roll", "pitch", "yaw"]):
        ax4.plot(t, err_deg[:, i], label=name)
    ax4.set_xlabel("time (s)")
    ax4.set_ylabel("estimation error (deg)")
    ax4.set_title("Madgwick Attitude Estimation Error")
    ax4.grid(alpha=0.3)
    ax4.legend()
    fig4.tight_layout()

    fig5 = plot_imu_data(t, log["gyro_meas"], log["accel_meas"],
                          title="Simulated MPU6050 IMU stream")

    return [fig1, fig2, fig3, fig4, fig5]


if __name__ == "__main__":
    log = run_pipeline(duration_s=10.0, dt=1.0 / 5000, seed=0, controller_type="pid")

    final_pos = log["pos"][-1]
    final_err = np.degrees(log["att_est"][-1] - log["att_true"][-1])
    print(f"Final position (x, y, alt): {np.round(final_pos, 3)} m")
    print(f"Setpoint (x, y, alt):       {log['pos_des']} m")
    print(f"Final attitude estimation error (roll, pitch, yaw): {np.round(final_err, 3)} deg")

    figs = plot_results(log)
    plt.show()