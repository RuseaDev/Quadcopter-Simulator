"""
PID has
kp: proportional control
ki: integral control
kd: derivative control
"""

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral = 0 # when you integrate over time, this will value will get bigger
        
    def update(self):
        return None

"""
The CascadedController will have the outer and the inner controller
The outer controller will output wx, wy and wz, which 
feed into the inner controller, which in turn outputs
tau_x, tau_y, tau_z, which feeds directly into 
plant.py 
"""
class CascadedController:
    def __init__(self):
        self.inner_controller = 0




    
