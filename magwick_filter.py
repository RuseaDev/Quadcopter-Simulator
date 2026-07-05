import numpy as np
from quaternions import Quaternion

class MadgwickMARG:
    """
    Madgwick filter using input data from gyroscope, accelerometer, and magnetometer.
    Quaternion order is [w, x, y, z]
    Gyroscope's unit has to be rad/s
    dt must be in seconds (calculated from the IMU freq)
    """

    def __init__ (self, beta = None, zeta = 0, q0 = None): 
        if beta is None: 
            gyro_error = np.deg2rad (5.0)
            beta = np.sqrt (3.0 / 4.0) * gyro_error

        # Convert to float in case the input is 'int'
        self.beta = float(beta)
        self.zeta = float (zeta)

        if q0 is None: 
            q0 = np.array ([1.0, 0.0, 0.0, 0.0])

        #normalize q (sqrt of square sum of all components = 1)
        self.q = self.normalize(np.array (q0, dtype = float))

        #initial magnetic reference (this is updated after each iteration)
        self.bx = 1.0
        self.bz = 0.0 

        #estimated gyroscope bias (rad / s)
        self.gyro_bias = np.zeros (3, dtype = float)
    
    def quaternion_to_array (q):
        e
