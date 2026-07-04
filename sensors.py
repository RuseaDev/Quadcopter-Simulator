"""
6-axis IMU 
Accelerometer + Gyroscope 
ax, ay, az      wx, wy, wz
"""

g = 9.81 # gravitational acceleration 

class Accelerometer:
    def __init__(self):
        self.true = 0
        self.bias = 0 # bias isn't constant and it doesn't update randomly. Does it acculumate forever?
        self.noise = 0 # There are different kinds of noise

    def read(self, Rb2w):
        """
        The accelerometer will return acceleration in its accelerometer frame
        (same with drone body frame in this case because they are glued together)

        The accelerometer will 

        """

        az = 
        ax = 

        

        # Rotation from body to world frame
        return ax, ay, az


