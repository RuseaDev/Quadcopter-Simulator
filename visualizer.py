import importlib
import plant
importlib.reload (plant)
import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt 
from matplotlib.backends.backend_pdf import PdfPages
from plant import DroneConfig, DronePlant, RK4_step, b2w_rotatation, g
from scipy.spatial.transform import Rotation as R

def default_input(t, config):
    hover = config.mass * g
    thrust = hover
    torques = np.zeros(3)

    if 0.0 <= t < 2.0:
        thrust = 1.2 * hover
        if 0.2 <= t < 0.8:
            torques[0] = 0.004
        elif 0.8 <= t < 1.4:
            torques[0] = -0.004

    elif 2.0 <= t < 5.0:
        thrust = hover

        if 2.2 <= t < 3.0:
            torques[1] = -0.004
        elif 3.0 <= t < 3.8:
            torques[1] = 0.004

        if 3.8 <= t < 4.4:
            torques[0] = -0.004
        elif 4.4 <= t < 5.0:
            torques[0] = 0.004

    elif 5.5 <= t < 9.0:
        thrust = 0.84 * hover
        if 5.5 <= t < 6.2:
            torques[1] = 0.004
        if 6.2 <= t < 6.9:
            torques[1] = -0.004

        if 5.5 <= t < 6.5:
            torques[0] = -0.006
        elif 6.5 <= t < 7.5:
            torques[0] = 0.006

    return thrust, torques

def simulate_trajectory (
    config, 
    y0, t0, t_end, dt,
    input_func = default_input
):
    
    def f (t, y): 
        thrust, torques = default_input(t, config)
        plant = DronePlant(config, y)
        return plant.state_derivatives(thrust=thrust, torques=torques)

    times = [t0]
    state_vec = [y0.copy()]
    t = t0
    y = y0.copy() 

    while t < t_end: 
        y = RK4_step (f, t, y, dt)
        t = t + dt
        times.append (t) 
        state_vec.append (y.copy())

    state_comp = [
        'x', 'y', 'z',
        'vx','vy','vz',
        'phi','theta','psi',
        'wx','wy','wz',
    ]

    df = pd.DataFrame(state_vec, columns = state_comp)
    df.insert(loc = 0, column = 'time', value = times)
    return df 

def plot_state (df: pd.DataFrame): 
    df = df.copy()
    fig, axs = plt.subplots (2, 2, figsize = (10, 10))
    axs[0, 0].plot (df['time'], df['x'], label = 'x') 
    axs[0, 0].plot (df['time'], df['y'], label = 'y') 
    axs[0, 0].plot (df['time'], -df['z'], label = 'height') 
    axs[0, 0].set_title ('Position')
    axs[0, 0].set_xticks (np.arange(0, df['time'].iloc[-1], 1))

    axs[0, 1].plot (df['time'], df['vx'], label = 'vx')
    axs[0, 1].plot (df['time'], df['vy'], label = 'vy')
    axs[0, 1].plot (df['time'], df['vz'], label = 'vz')
    axs[0, 1].set_title ('Velocity')
    axs[0, 1].set_xticks (np.arange(0, df['time'].iloc[-1], 1))

    axs[1, 0].plot (df['time'], df['phi'], label = 'phi')
    axs[1, 0].plot (df['time'], df['theta'], label = 'theta')
    axs[1, 0].plot (df['time'], df['psi'], label = 'psi')
    axs[1, 0].set_title ('Euler Angles')
    axs[1, 0].set_xticks (np.arange(0, df['time'].iloc[-1], 1))

    axs[1, 1].plot (df['time'], df['wx'], label = 'wx')
    axs[1, 1].plot (df['time'], df['wy'], label = 'wy')
    axs[1, 1].plot (df['time'], df['wz'], label = 'wz')
    axs[1, 1].set_title ('Angular Velocity')    
    axs[1, 1].set_xticks (np.arange(0, df['time'].iloc[-1], 1))
    
    for ax in axs.flatten(): 
        ax.legend()
        ax.grid (True)
    axs[1, 0].set_xlabel ('Time')
    axs[1, 1].set_xlabel ('Time')
    fig.suptitle('Stat 2D Plot')

    return fig, axs

def plot_trajectory (df: pd.DataFrame): 
    fig = plt.figure (figsize = (10, 10)) 
    ax = fig.add_subplot(projection = '3d') 

    ax.plot (df['x'], df['y'], -df['z'], alpha = 0.8)

    ax.scatter (
        df['x'].iloc[0], 
        df['y'].iloc[0],
        -df['z'].iloc[0], 
        label = 'start',
    )
    ax.scatter (
        df['x'].iloc[-1],
        df['y'].iloc[-1],
        -df['z'].iloc[-1],
        label = 'end', 
    )

    ax.set_xlabel ('x')
    ax.set_ylabel ('y')
    ax.set_zlabel ('height')
    ax.legend() 
    fig.suptitle('3D State Plot')

    return fig, ax

def plot_gimbal_lock (): 
    theta_l = np.linspace(np.deg2rad(70), np.deg2rad(89.90), 200)
    theta_r = np.linspace(np.deg2rad(90.10), np.deg2rad(110), 200)
    theta_rad = np.concatenate([theta_l, theta_r])
    theta_deg = np.rad2deg(theta_rad)

    phi_input_deg = 10.0
    psi_input_deg = 20.0
    phi_input_rad = np.deg2rad(phi_input_deg)
    psi_input_rad = np.deg2rad(psi_input_deg)

    wx = np.ones_like(theta_rad)
    wy = 0.2 * np.ones_like(theta_rad)
    wz = 0.5 * np.ones_like(theta_rad)

    phi_dot = []
    theta_dot = []
    psi_dot = []

    for theta, wx_i, wy_i, wz_i in zip(theta_rad, wx, wy, wz):
        euler_angle_matrix = np.array([
            [1, np.sin(phi_input_rad) * np.tan(theta), np.cos(phi_input_rad) * np.tan(theta)],
            [0, np.cos(phi_input_rad), -np.sin(phi_input_rad)],
            [0, np.sin(phi_input_rad) / np.cos(theta), np.cos(phi_input_rad) / np.cos(theta)],
        ])

        euler_dot = euler_angle_matrix @ np.array([wx_i, wy_i, wz_i])
        phi_dot.append(euler_dot[0])
        theta_dot.append(euler_dot[1])
        psi_dot.append(euler_dot[2])

    phi_dot = np.array(phi_dot)
    theta_dot = np.array(theta_dot)
    psi_dot = np.array(psi_dot)

    cos_theta_inverse = np.abs(1 / np.cos(theta_rad))

    phi_recover = []
    theta_recover = []
    psi_recover = []

    for theta in theta_rad:
        rotational_matrix = b2w_rotatation(phi_input_rad, theta, psi_input_rad)
        convert = R.from_matrix(rotational_matrix)
        phi_rec, theta_rec, psi_rec = convert.as_euler('ZYX')

        phi_recover.append(phi_rec)
        theta_recover.append(theta_rec)
        psi_recover.append(psi_rec)

    phi_recover = np.rad2deg(phi_recover)
    theta_recover = np.rad2deg(theta_recover)
    psi_recover = np.rad2deg(psi_recover)

    fig, axs = plt.subplots(2, 2, figsize=(10, 10))

    axs[0, 0].plot(theta_deg, cos_theta_inverse, label='1 / cos(theta)')
    axs[0, 0].set_title('1 / cos (theta)')
    axs[0, 0].legend()
    axs[0, 0].grid(True)

    axs[0, 1].plot(theta_deg, np.rad2deg(phi_dot), label='dphi')
    axs[0, 1].plot(theta_deg, np.rad2deg(theta_dot), label='dtheta')
    axs[0, 1].plot(theta_deg, np.rad2deg(psi_dot), label='dpsi')
    axs[0, 1].set_ylabel('Euler rate (deg/s)')
    axs[0, 1].set_title('Euler Angle Derivatives')
    axs[0, 1].legend()
    axs[0, 1].grid(True)

    axs[1, 0].plot(theta_deg, np.full_like(theta_deg, psi_input_deg), label='input roll')
    axs[1, 0].plot(theta_deg, theta_deg, label='input pitch')
    axs[1, 0].plot(theta_deg, np.full_like(theta_deg, phi_input_deg), label='input yaw')
    axs[1, 0].set_xlabel('Input  theta (deg)')
    axs[1, 0].set_ylabel('Degree')
    axs[1, 0].set_title('Input Euler Angles')
    axs[1, 0].legend()
    axs[1, 0].grid(True)

    axs[1, 1].plot(theta_deg, phi_recover, label='recover roll')
    axs[1, 1].plot(theta_deg, theta_recover, label='recover pitch')
    axs[1, 1].plot(theta_deg, psi_recover, label='recover yaw')
    axs[1, 1].set_xlabel('Input theta (deg)')
    axs[1, 1].set_ylabel('Recover angle (deg)')
    axs[1, 1].set_title('Recover Euler Angles from Rotation Matrix')
    axs[1, 1].legend(loc = 'lower left')
    axs[1, 1].grid(True)
    
    fig.suptitle('Gimbal Lock Problems')
    fig.subplots_adjust(wspace=0.3)
    return fig, axs

def run(
    mass = 2.0,
    inertia = np.diag([0.01, 0.01, 0.02]),
    length = 0.2,
    kd = 0, kt = 0, kb = 0,
    y0 = np.zeros(12), 
    dt = 0.02, 
    time_length = 10.0,
): 
    
    config = DroneConfig(mass = mass, inertia=inertia, length=length, kd = kd, kt = kt, kb = kb)
    df = simulate_trajectory (
        config = config, 
        y0 = y0,
        t0 = 0.0, 
        t_end = time_length,
        dt = dt, 
        input_func = default_input
    )

    fig_state, _ = plot_state(df)
    fig_traj, _ = plot_trajectory(df)
    fig_gimbal, _ = plot_gimbal_lock()

    with PdfPages("visualizer_image.pdf") as pdf:
        pdf.savefig(fig_state)
        pdf.savefig(fig_traj)
        pdf.savefig(fig_gimbal)
    
if __name__ == '__main__': 
    run()