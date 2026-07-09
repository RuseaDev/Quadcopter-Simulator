import numpy as np
from dataclasses import dataclass, field
from scipy.signal import lfilter
import matplotlib.pyplot as plt

G = 9.81

@dataclass
class AxisNoiseParams:
    N: float        # white-noise density, unit/sqrt(Hz)
    B: float        # Gauss-Markov bias std, same unit as sensor output
    tau_c: float    # bias correlation time, s

# Real measured parameters (Gonzalez & Dabove 2019, Table 2), per axis [X, Y, Z]
MPU6050_GYRO_DEFAULT = [
    AxisNoiseParams(N=6.625e-5, B=1.674e-5, tau_c=900.0),  # X, rad/s/sqrt(Hz), rad/s, s
    AxisNoiseParams(N=5.934e-5, B=2.301e-5, tau_c=200.0),  # Y
    AxisNoiseParams(N=6.050e-5, B=1.462e-5, tau_c=200.0),  # Z
]
MPU6050_ACCEL_DEFAULT = [
    AxisNoiseParams(N=1.156e-3, B=3.703e-4, tau_c=30.0),   # X, (m/s^2)/sqrt(Hz), m/s^2, s
    AxisNoiseParams(N=1.252e-3, B=2.501e-4, tau_c=300.0),  # Y
    AxisNoiseParams(N=1.820e-3, B=5.058e-4, tau_c=200.0),  # Z
]

def gauss_markov_bias(n, dt, B, tau_c, rng):
    """
    First-Order Gauss-Markov bias
    Ornstein-Uhlenbeck process: db/dt = -b/tau + noise
    with stationary std B and correlation time tau_c.
    """

    # if there is no standard deviation, then bias would be zero for all values
    if B <= 0:
        return np.zeros(n)
    
    phi = np.exp(-dt / tau_c)
    q = B * np.sqrt(1.0 - phi ** 2) # q = B*sqrt(1-e^(2*(-dt/tau_c))

    # Throw away some initial values to let the bias settle into steady state
    burn_in = int(5 * tau_c / dt)
    white = rng.standard_normal(burn_in + n)
    return lfilter([q], [1.0, -phi], white)[burn_in:]

@dataclass
class MPU6050Config:
    sample_rate_hz: float = 100.0
    gyro_params: list = field(default_factory=lambda: MPU6050_GYRO_DEFAULT)
    accel_params: list = field(default_factory=lambda: MPU6050_ACCEL_DEFAULT)

    gyro_constant_bias_rad_s: np.ndarray = field(default_factory=lambda: np.zeros(3))
    accel_constant_bias_mps2: np.ndarray = field(default_factory=lambda: np.zeros(3))
    seed: int = None

class MPU6050Simulator:
    def __init__(self, config: MPU6050Config):
        self.config = config
        self.rng = np.random.default_rng(self.config.seed)

    def simulate(self, true_gyro_rad_s, true_accel_mps2, duration_s):
        """
        true_gyro_rad_s, true_accel_mps2: (3,) constant vector
        
        Retruns (t, gyro_meas_rad_s, accel_meas_mps2)"""

        fs = self.config.sample_rate_hz # Frequency of the sample
        dt = 1.0 / fs # f = 1/T -> T = 1/f

        true_gyro_rad_s = np.asarray(true_gyro_rad_s)
        true_accel_mps2 = np.asarray(true_accel_mps2)

        if true_gyro_rad_s.ndim == 1:
            n = int(round(duration_s * fs))
            true_gyro_rad_s = np.tile(true_gyro_rad_s, (n,1))
            true_accel_mps2 = np.tile(true_accel_mps2, (n,1))
        else:
            n = true_gyro_rad_s.shape[0]

        t = np.arange(n) * dt
        gyro_meas = np.empty((n,3))
        accel_meas = np.empty((n, 3))

        for axis in range(3):
            gp = self.config.gyro_params[axis] # Gyro param
            gyro_white = gp.N * np.sqrt(fs) * self.rng.standard_normal(n)
            gyro_bias = gauss_markov_bias(n, dt, gp.B, gp.tau_c, self.rng)
            gyro_meas[:, axis] = (true_gyro_rad_s[:, axis]
                                  + self.config.gyro_constant_bias_rad_s[axis]
                                  + gyro_white + gyro_bias)
            
            ap = self.config.accel_params[axis] # Accel param
            accel_white = ap.N * np.sqrt(fs) * self.rng.standard_normal(n)
            accel_bias = gauss_markov_bias(n, dt, ap.B, ap.tau_c, self.rng)
            accel_meas[:, axis] = (true_accel_mps2[:, axis]
                                   + self.config.accel_constant_bias_mps2[axis]
                                   + accel_bias + accel_white)
            
        return t, gyro_meas, accel_meas
        
    def simulate_static(self, duration_s, level_accel_mps2=(0.0, 0.0, G)):
        """
        For convenience, we'll set the IMU at rest, ground level 0, and
        let gravity be the only force acting on the accelerometer.
        """
        return self.simulate(
            true_gyro_rad_s = np.zeros(3),
            true_accel_mps2=np.array(level_accel_mps2),
            duration_s=duration_s
        )

def plot_imu_data(t, gyro, accel, title="MPU6050 simulated data"):
    """
    Time-series plot of simulated gyro (rad/s) and accel (m/s^2)
    """

    axis_labels = ["X", "Y", "Z"]
    fig, axes = plt.subplots(3, 2, figsize = (10, 8), sharex = True)

    for i in range(3):
        axes[i, 0].plot(t, gyro[:, i], color="C0")
        axes[i, 0].set_ylabel(f"gyro {axis_labels[i]}\n(rad/s)")
        axes[i, 0].grid(alpha=0.3)

        axes[i, 1].plot(t, accel[:, i], color="C1")
        axes[i, 1].set_ylabel(f"accel {axis_labels[i]}\n(m/s^2)")
        axes[i, 1].grid(alpha=0.3)


    axes[-1, 0].set_xlabel("time (s)")
    axes[-1, 1].set_xlabel("time (s)")

    fig.suptitle(title)
    fig.tight_layout()
    return fig

if __name__ == "__main__":
    sim = MPU6050Simulator(MPU6050Config(sample_rate_hz=100.0, seed=42))
    t, gyro, accel = sim.simulate_static(duration_s=10.0)

    fig = plot_imu_data(t, gyro, accel, title="MPU6050 simulated static data (10 s)")
    fig.savefig("mpu6050_simulatd_data.png", dpi=150)
    