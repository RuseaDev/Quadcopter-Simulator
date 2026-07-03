import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches

class PID:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, current_value, dt):
        error = self.setpoint - current_value

        p = self.kp * error

        self.integral += error * dt
        i = self.ki * self.integral

        derivative = (error - self.prev_error) / dt
        d = self.kd * derivative

        self.prev_error = error

        return p + i + d

