import importlib
import plant
importlib.reload (plant)
import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt 
from plant import DroneConfig, DronePlant, RK4_step, b2w_rotatation, g

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
    fig, axs = plt.subplots (2, 2, figsize = (8, 5))
    axs[0, 0].plot (df['time'], df['x'], label = 'x') 
    axs[0, 0].plot (df['time'], df['y'], label = 'y') 
    axs[0, 0].plot (df['time'], -df['z'], label = 'height') 
    axs[0, 0].set_ylabel ('Position')

    axs[0, 1].plot (df['time'], df['vx'], label = 'vx')
    axs[0, 1].plot (df['time'], df['vy'], label = 'vy')
    axs[0, 1].plot (df['time'], df['vz'], label = 'vz')
    axs[0, 1].set_ylabel ('Velocity')

    axs[1, 0].plot (df['time'], df['phi'], label = 'phi')
    axs[1, 0].plot (df['time'], df['theta'], label = 'theta')
    axs[1, 0].plot (df['time'], df['psi'], label = 'psi')
    axs[1, 0].set_ylabel ('Euler Angles')

    axs[1, 1].plot (df['time'], df['wx'], label = 'wx')
    axs[1, 1].plot (df['time'], df['wy'], label = 'wy')
    axs[1, 1].plot (df['time'], df['wz'], label = 'wz')
    axs[1, 1].set_ylabel ('Angular Velocity')    
    
    for ax in axs.flatten(): 
        ax.legend()
        ax.grid (True)
    axs[1, 0].set_xlabel ('Time')
    axs[1, 1].set_xlabel ('Time')

    return fig, axs 

def plot_trajectory (df: pd.DataFrame): 
    fig = plt.figure (figsize = (8, 8)) 
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

    return fig, ax

def plot_gimble_lock (): 
    
    return 

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
    plot_state (df)
    plot_trajectory(df)
    plt.show()

if __name__ == '__main__': 
    run()