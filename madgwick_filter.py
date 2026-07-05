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
    
    @staticmethod
    def quaternion_to_array (q):
        return np.array ([q.w, q.x, q.y, q.z])
    
    @staticmethod 
    def array_to_quaternion(q): 
        qw, qx, qy, qz = np.asarray(q, dtype = float)
        return Quaternion(qw, qx, qy, qz)

    @classmethod
    def normalize (cls, q): 
        normalized = cls.array_to_quaternion(q).normalize()
        return cls.quaternion_to_array(normalized)
    
    @classmethod 
    def quaternion_product (cls, q, r): 
        product = cls.array_to_quaternion (q) * cls.array_to_quaternion(r)
        return cls.quaternion_to_array(product)

    @classmethod
    def quaternion_conjugate (cls, q): 
        conjugate = cls.array_to_quaternion(q).conjugate()
        return cls.quaternion_to_array(conjugate)
    
    @staticmethod
    def madgwick_jacobian (q, bx, bz): 
        q1, q2, q3, q4 = q
        two_bx = 2.0 * bx
        two_bz = 2.0 * bz

        return np.array([
            [-2.0*q3, 2.0*q4, -2.0*q1, 2.0*q2],
            [2.0*q2, 2.0*q1, 2.0*q4, 2.0*q3],
            [0.0, -4.0*q2, -4.0*q3, 0.0],
            [
                -two_bz*q3,
                two_bz*q4,
                -4.0*bx*q3 - two_bz*q1,
                -4.0*bx*q4 + two_bz*q2,
            ],
            [
                -two_bx*q4 + two_bz*q2,
                two_bx*q3 + two_bz*q1,
                two_bx*q2 + two_bz*q4,
                -two_bx*q1 + two_bz*q3,
            ],
            [
                two_bx*q3,
                two_bx*q4 - 4.0*bz*q2,
                two_bx*q1 - 4.0*bz*q3,
                two_bx*q2,
            ],
        ])
    
    def integrate_gyro_only(self, gyro, dt): 
        """
        This helper function is used when either the normalized value of magnetometer or accelerometer = 0 (causing division to zero). Then the filter will update quaternion with data from gyroscope
        """

        gyro = np.asarray (gyro, dtype = float)
        if self.zeta != 0:
            gyro = gyro - self.gyro_bias
        
        q_dot = 0.5 * self.quaternion_product(
            self.q, 
            np.array ([0.0, gyro[0], gyro[1], gyro[2]]),
        )
        self.q = self.normalize(self.q + q_dot * dt)
        return self.q

    def update_magnetic_reference (self, mag_unit): 
        """
        Filter will use this func to update the bx and bz paramters of the magnetic field.
        """
        mag_quat = np.array ([0.0, mag_unit [0], mag_unit[1], mag_unit[2]])
        h = self.quaternion_product(
            self.quaternion_product(self.q, mag_quat), 
            self.quaternion_conjugate(self.q),
        )

        hx, hy, hz = h[1], h[2], h[3]
        self.bx = np.sqrt (hx * hx + hy * hy)
        self.bz = hz

    
    def update (self, gyro, accel, mag, dt): 
        """
        This function runs each update of the filter
        
        Param: 
            gyro: angular velocity in rad / s
            accel: acceleratation
            mag: magnetic field

        Output:
            updated quaternion
        """

        gyro = np.asarray (gyro, dtype = float)
        accel = np.asarray (accel, dtype = float)
        mag = np.asarray (mag, dtype = float)
        dt = float (dt) 

        if dt <= 0.0: 
            return self.q.copy()
        
        accel_norm = np.linalg.norm (accel)
        mag_norm = np.linalg.norm (mag)

        if accel_norm == 0.0 or mag_norm == 0.0: 
            return self.integrate_gyro_only(gyro, dt)
        
        ax, ay, az = accel / accel_norm 
        mx, my, mz = mag / mag_norm 

        q1, q2, q3, q4 = self.q
        bx, bz = self.bx, self.bz

        two_q1 = 2.0 * q1
        two_q2 = 2.0 * q2
        two_q3 = 2.0 * q3
        two_q4 = 2.0 * q4
        two_bx = 2.0 * bx
        two_bz = 2.0 * bz

        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q4 = q3 * q4

        #Objective function: predicted gravity / magnetic directions minus measured directions

        f1 = two_q2*q4 - two_q1*q3 - ax
        f2 = two_q1*q2 + two_q3*q4 - ay
        f3 = 1.0 - two_q2*q2 - two_q3*q3 - az
        f4 = two_bx*(0.5 - q3*q3 - q4*q4) + two_bz*(q2q4 - q1q3) - mx
        f5 = two_bx*(q2q3 - q1q4) + two_bz*(q1q2 + q3q4) - my
        f6 = two_bx*(q1q3 + q2q4) + two_bz*(0.5 - q2*q2 - q3*q3) - mz

        objective = np.array ([f1, f2, f3, f4, f5, f6])
        jacobian = self.madgwick_jacobian(self.q, bx, bz)
        gradient = jacobian.T @ objective 
        gradient_norm = np.linalg.norm (gradient)

        if gradient_norm == 0:
            return self.integrate_gyro_only(gyro, dt)
        
        gradient = gradient / gradient_norm 
        s1, s2, s3, s4 = gradient 

        if self.zeta != 0.0:
            # Convert quaternion error direction into angular error.
            w_err_x = two_q1*s2 - two_q2*s1 - two_q3*s4 + two_q4*s3
            w_err_y = two_q1*s3 + two_q2*s4 - two_q3*s1 - two_q4*s2
            w_err_z = two_q1*s4 - two_q2*s3 + two_q3*s2 - two_q4*s1

            self.gyro_bias += self.zeta * np.array([w_err_x, w_err_y, w_err_z]) * dt
            gyro = gyro - self.gyro_bias

        wx, wy, wz = gyro

        #Quaternion derivative from gyroscope 
        q_dot_gyro = 0.5 * self.quaternion_product(self.q, np.array ([0.0, wx, wy, wz]))

        #Correct gyro derivative with accelerometer & magnetometer gradient 

        q_dot = q_dot_gyro - self.beta * gradient

        #integrate and normalize quaternion
        self.q = self.normalize (self.q + q_dot * dt)

        #update magnetic reference using the latest orientation
        self.update_magnetic_reference(np.array ([mx, my, mz]))

        return self.q
    
    def quaternion_to_euler (self, q):
        qw, qx, qy, qz = q

        #Roll around axis x
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = np.arctan2 (sinr_cosp, cosr_cosp)

        #Pitch around axis y 
        sinp = 2.0 * (qw * qy - qz * qx)
        sinp = np.clip (sinp, -1.0, 1.0)
        pitch = np.arcsin (sinp)

        #Yaw around axis z
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy* qy + qz * qz)
        yaw = np.arctan2 (siny_cosp, cosy_cosp)

        return np.array ([roll, pitch, yaw])