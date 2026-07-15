"""
main.py

This is the full pipeline for the drone:

Full closed-loop simulation:

DronePlant (the truth)

thrust/torques -> true 12-state trajectory
add sensor noise -> """

import numpy as np
from plant import DroneConfig, DronePlant, g, b2w_rotatation
from mpu6050_imu_sim import MPU6050_GYRO_DEFAULT, MPU6050_ACCEL_DEFAUL, AxisNoiseParams
from madgwick_filter import MadgwickMARG
from pid_controller import CascadedController


def run_pipeline(duration_s=10.0, dt = 1.0/500, seed = 0):
    config = DroneConfig(
        mass = 1.2,
        inertia = np.diag([0.02, 0.02, 0.04]),
        length = 0.25
        kd = 0.01,
        kt = 3e-5
        kb=1e-6
    )
    
    # Truth will be tipped over, off-center, spinning start
    x0 = np.zeros(12)
    x0[0:3] = [1.0, -0.5, -2.0]     # NED: negative z = 2 m up
    x0[6:9] = [0.3, -0.2, 0.5]      # roll, pitch, yaw offset (rad)
    x0[9:12] = [0.1, -0.1, 0.05]    # initial body rates (rad/s)
    plant = DronePlant(config, x0.copy())

    controller = CascadedController(config, outer_rate_hz=50, attitude_rate_hz=1.0/dt)
    madgwick = MadgwickMARG(zeta=0.03)

    pos_des = np.array([0.0, 0.0, 2.0])
    psi_des = 0.0

    n_steps = int(round(duration_s / dt))
  
