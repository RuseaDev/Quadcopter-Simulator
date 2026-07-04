import numpy as np
from plant import DroneConfig, DronePlant

g = 9.81 # Gravity m/s^2

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

"""
Cascaded PID:

The PID above takes a setpoint at initialization and computes error
internally as setpoint - current_value. 

The cascade needs to change each PID's target every loop tick 
(The outer loop keeps feeding new attitude targets to the inner loop)

PID above does not yet clamp the integral to its maximum

"""

# Outer Controller
class PositionController:
    """
    Input: desired (x, y, altitude (z), yaw) 
    vs 
    current position

    Output: desired (phi, theta, psi) and total thrust

    Horizontal control uses the standard small-angle approximation
    (It is valid near hover)

    A desired world-frame horizontal acceleration is rotated into the body heading frame
    and mapped to a tilt angle. Altitude control (inner controller)
    computes a desired vertical acceleration and converts it to thrust.
    """

    MAX_TILT = np.radians(20)

    #Altitude clamp
    ALT_ACCEl_LIMIT = 10.0
    XY_ACCEL_LIMIT = 4.0
    ALT_INTEGRAL_LIMIT = 3.0
    XY_INTEGRAL_LIMIT = 3.0

    def __init__(self, config):
        self.config = config
        self.pid_altitude = PID(kp=8, ki=2, kd=5, setpoint = 0)
        self.pid_x = PID(kp = 0.6, ki = 0.02, kd = 0.9, setpoint = 0)
        self.pid_y = PID(kp= 0.6, ki=0.02, kd = 0.9, setpoint=0)


    def update(self, pos_des, psi_des, plant: DronePlant, dt):
        x_des, y_des, alt_des = pos_des
        x, y, z = plant.position
        # convert NED z (down+) to altitude (up+)
        altitude = -z
        phi = plant.euler_angles[0]
        theta = plant.euler_angles[1]

        m = plant.config.mass
        rotation = np.cos(phi) * np.cos(theta)

        self.pid_attitude.setpoint = alt_des
        accel_cmd = self.pid_altitude.update(altitude, dt)

        thrust = (m * (g + accel_cmd)) / rotation
        self.pid_x.setpoint = x_des
        theta = self.pid_x.update(-x, dt)

        self.pid_y.setpoint = y_des
        phi = self.pid_y.update(y, dt)

        return np.array([phi, theta, psi_des]), thrust

class AttitudeController:
    """
    Input: desired pitch, roll, and thrust
    Output: angular rates (w_x, w_y, w_z)
    """
    def __init__(self, config):
        self.pid_phi = PID(kp=6.0, ki=0.0, kd = 0, setpoint=0)
        self.pid_theta = PID(kp=6.0, ki=0.0, kd = 0, setpoint=0)
        self.pid_psi = PID(kp=6.0, ki=0.0, kd = 0, setpoint=0)
    
    def update(self, att_des, plant: DronePlant, dt):
        phi_des, theta_des, psi_des = att_des
        phi, theta, psi = plant.euler_angles

        self.pid_phi.setpoint = phi_des
        w_x = self.pid_phi.update(phi, dt)

        self.pid_theta.setpoint = theta_des
        w_y = self.pid_theta.update(theta, dt)

        self.psi_psi.setpiont = psi_des
        w_z = self.pid_psi.update(psi, dt)

        

class RateController:
    """
    Input: desired w_x, w_y, w_z
    Output: torques (tau_x, tau_y, tau_z)
    """
    def __init__(self, config):
        self.pid_wx = PID(kp = 0, ki = 0, kd = 0, setpoint = 0)
        self.pid_wy = PID(kp = 0, ki = 0, kd = 0, setpoint = 0)
        self.pid_wz = PID(kp = 0, ki = 0, kd = 0, setpoint = 0)


    def update(self, att, plant: DronePlant, dt):
        wx_des, wy_des, wz_des = att
        wx, wy, wz = plant.angular_velocity
        
        self.pid_wx.setpoint = wx_des
        tau_x = self.pid_wx.update(wx, dt)

        self.pid_wy.setpoint = wy_des
        tau_y = self.pid_wy.update(wy, dt)

        self.pid_wz.setpoint = wz_des
        tau_z = self.pid_wz.update(wz, dt)


        return np.array([tau_x, tau_y, tau_z])
    
class CascadedController:

    def __init__(self, config, outer_rate_hz=50, attitude_rate_hz = 500)
        self.position_control = PositionController(config)
        self.attitude_control = AttitudeController(config)
        self.rate_control = RateController(config)


        self.outer_period = 1.0 / outer_rate_hz
        self.attitude_period = 1.0 / attitude_rate_hz

        self.t_since_outer = np.inf
        self.t_since_attitude = np.inf

        # Initializing desired attitude and rate to [0, 0 ,0]
        self.att_des = np.zeros(3)
        self.thrust = config.mass * g
        self.rate_des = np.zeros(3)
    
    def update(self, pos_des, psi_des, plant: DronePlant, dt):
        self.t_since_outer += dt
        self.t_since_attitude += dt

        if self.t_since_outer >= self.outer_period:
            self.att_des, self.thrust = self.position_control.update(
                pos_des, psi_des, plant, self.outer_period)
            self.t_since_outer = 0.0
        
        if self.t_since_attitude >= self.attitude_period:
            self.rate_des = self.attitude_control.updae(
                self.att_des, plant, self.attitude_period)
            self.t_since_attitude = 0.0
        
        torques = self.rate.ctrl.update(self.rate_des, plant, dt)
        return self.thrust, torques


        
