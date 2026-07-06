"""
MPU6050 IMU simulator grounded in real Allan-variance characterization

It implements the standard IEEE-STD-952 model used to characterize
the real gyros and accelerometers (angle/velocity random walk + bias instability,
via Allan variance), populated with actual measured noise parameters for the MPU-6050 sensor die
and it includes an Allan-deviation analyzer that can re-derive those noise parameters
from any data you feed it.

Data sources:
. Gonzalez, R.; Dabove, P. "Performance Assessment of an Ultra Low-Cost
   Inertial Measurement Unit for Ground Vehicle Navigation." Sensors 2019,
   19, 3865. https://doi.org/10.3390/s19183865
   -> 24-hour static Allan variance analysis of one MPU-6000 unit at 200 Hz.
   Table 2 gives random walk (N), dynamic bias / bias instability (B), and
   correlation time for all 3 gyro axes and all 3 accel axes, in SI units.
   This is the PRIMARY default used below (MPU6050_GYRO_DEFAULT /
   MPU6050_ACCEL_DEFAULT).
 
2. Rudyk, A.V. et al. "Strapdown Inertial Navigation Systems for
   Positioning Mobile Robots -- MEMS Gyroscopes Random Errors Analysis
   Using Allan Variance Method." Sensors 2020, 20, 4841.
   https://doi.org/10.3390/s20174841
   -> Separate 1-hour static Allan variance analysis of an MPU-6050 gyro at
   70 Hz. Included as MPU6050_GYRO_RUDYK2020 to show unit-to-unit /
   study-to-study variation is real for MEMS parts -- don't expect your
   specific chip to match either dataset exactly.
 
3. InvenSense "MPU-6000 and MPU-6050 Product Specification" Rev 3.4 (2013),
   pp. 12-13: datasheet noise spectral density is 0.005 deg/s/sqrt(Hz) for
   the gyro and 400 ug/sqrt(Hz) for the accelerometer. Included as
   MPU6050_GYRO_DATASHEET / MPU6050_ACCEL_DATASHEET -- this is what you'd
   use if you had no bench data at all, but it only gives you the white
   noise (N) term, not bias instability, since manufacturers don't
   typically characterize the flicker floor on the datasheet.
 
Everything internally uses SI units: rad/s for gyro, m/s^2 for accel,
seconds for time.
"""

import numpy as np
from dataclasses import dataclass
from scipy.signal import lfilter

G = 9.80665 
DEG2RAD = np.pi / 180.0

@dataclass 
class AxisNoiseParams:
    """
    N: random walk coefficient (ARW for gyro, VRW for accel)
    
    B: bias instability magnitude, in the same base unit as the sensor output
       (rad/s for gyrp, m/s^2 for accel).
       
    tau_c = correlation time of the Gauss-Markov process
    
    K: rate random walk coefficient
    """

    N: float
    B: float
    tau_c: float
    K: float = 0.0

# Real, published parameter sets

# Source 1: (Gonzalez & Dabove 2019, Table 2), gyro
# N in rad/s/sqrt(Hz), B in rad/s, tau_c in s.
MPU6050_GYRO_DEFAULT = [
    AxisNoiseParams(N=6.625e-5, B=1.674e-5, tau_c=900.0),  # X
    AxisNoiseParams(N=5.934e-5, B=2.301e-5, tau_c=200.0),  # Y
    AxisNoiseParams(N=6.050e-5, B=1.462e-5, tau_c=200.0),  # Z
]

# Source 1, accelerometer, per axis [X, Y, Z]
# N in m/s^2/sqrt(Hz), B in m/s^2, tau_c in s
MPU6050_ACCEL_DEFAULT = [
    AxisNoiseParams(N=1.156e-3, B=3.703e-4, tau_c=30.0),   # X
    AxisNoiseParams(N=1.252e-3, B=2.501e-4, tau_c=300.0),  # Y
    AxisNoiseParams(N=1.820e-3, B=5.058e-4, tau_c=200.0),  # Z
]


# Source 2 (Rudyk et al. 2020), gyro only, per axis[X, Y, Z]
MPU6050_GYRO_RUDYK2020 = [
    AxisNoiseParams(N=1.596e-4, B=4.17e-5, tau_c=60.0),   # X (0.009145 deg/s/sqrtHz, 0.00239 deg/s)
    AxisNoiseParams(N=1.745e-4, B=2.23e-5, tau_c=60.0),   # Y (0.009997 deg/s/sqrtHz, 0.001277 deg/s)
    AxisNoiseParams(N=1.664e-4, B=4.27e-5, tau_c=60.0),   # Z (0.009533 deg/s/sqrtHz, 0.002449 deg/s)
]