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

    

