""" 
We will have to build helper functions for quaternions, gradient descent, Jacobian matrix, etc
"""


"""
The Madgwick filter fuses accelerometer, gyroscope and magnetometer to estimate
orientation as a quaternion
"""
class MadgwickFilter:
    def __init__(self):
        self.quaternion = 0

    def update(self):
        return self.quaternion

    
import numpy as np 
# Madgwick filter
# Will use quaternions and gradient descent
# May use Kalman Filter later

# Use Complementary filter for the baseline
class MadgwickFilter:
    def __init__ (self, beta = 0.05, dt = 0.01):
        self.beta = beta #this = sqrt(3 / 4) * gyroscope zero measurement error -> check in the future
        self.dt = dt #in the future, this should be calculated from the freq of IMU
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

    def update (self, gyroscope_data, accelerometer_data):
        #gyroscope should be in rad / s
        
        accelerometer_data_norm = accelerometer_data / np.linalg.norm(accelerometer_data)

        q1, q2, q3, q4 = self.q
        ax, ay, az = accelerometer_data_norm
        gx, gy, gz = gyroscope_data

        gravity_objective_func = np.array(
            [2 * (q2 * q4 - q1 * q3) - ax, 2 * (q1 * q2 + q3 * q4) - ay, 2 * (1 / 2 - q2 ** 2 - q3 ** 2) - az]
        )

        f1, f2, f3 = gravity_objective_func

        direction = np.array (
            [2*q2*f2 - 2*q3*f1,
            2*q4*f1 + 2 * q1 * f2 - 4 * q2 * f3,
            2*q4*f2 - 4 * q3 * f3 - 2 * q1 * f1,
            2 *q2*f1 + 2*q3*f2], 
        )
        
        norm_direction = np.linalg.norm (direction)
        if norm_direction < 1e-7:
            direction_norm = np.zeros(4)
        else:
            direction_norm = direction / norm_direction
        qw = np.array (
            [-1 / 2 * q2 * gx - 1 / 2 * q3 * gy - 1 / 2 * q4 * gz, 
            1 / 2 * q1 * gx + 1 / 2 * q3 * gz - 1/ 2 * q4 * gy, 
            1 / 2 * q1 * gy - 1 / 2 * q2 * gz  + 1 / 2 * q4 * gx,
            1 / 2 * q1 * gz  + 1 / 2 * q2 * gy - 1 / 2 * q3 * gx], 
        )

        q_dot = qw - self.beta * direction_norm
        self.q = self.q + q_dot * self.dt
        self.q = self.q / np.linalg.norm (self.q)

        return self.q

    def quat2euler (self, q):
        q1, q2, q3, q4 = q.copy()

        roll = np.arctan2(
            2 * (q1 * q2 + q3 * q4),
            1 - 2 * (q2 ** 2 + q3 ** 2)
        )

        pitch = np.arcsin(np.clip(
            2 * (q1 * q3 - q4 * q2),
            -1.0,
            1.0
        ))

        yaw = np.arctan2(
            2 * (q1 * q4 + q2 * q3),
            1 - 2 * (q3 ** 2 + q4 ** 2)
        )

        return roll, pitch, yaw
