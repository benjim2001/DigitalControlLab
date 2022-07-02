import time
import math
from typing import Union

class PID(object):
    def __init__(self, p_gain: float, i_gain: float, d_gain: float, i_max: float, i_min: float):
        self.set_gains(p_gain, i_gain, d_gain, i_max, i_min)
        self.reset()
    def set_gains(self, p_gain: float, i_gain: float, d_gain: float, i_max: float, i_min: float): 
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.i_max = i_max
        self.i_min = i_min
    def reset(self):
        self.set_pstate = 0.0 
        self.set_perr = 0.0 
        self.set_derr = 0.0 
        self.set_ierr = 0.0
        self.cmd = 0.0 
        self.lt = None
    def update(self, set_perr: float, dt: Union[float, None] = None) -> float:
        if dt == None:
            cur_time = time.time()
            if self.lt is None:
                self.lt = cur_time 
            dt = cur_time - self.lt
            self.lt = cur_time    
        self.set_perr = set_perr # this is pError = pState-pTarget
        if dt == 0 or math.isnan(dt) or math.isinf(dt):
            return 0.0

        # Calculate proportional contribution to command
        p_term = self.p_gain * self.set_perr

        # Calculate the integral error
        self.set_ierr += dt * self.set_perr
        
        # Calculate integral contribution to command
        i_term = self.i_gain * self.set_ierr
        
        # Limit i_term so that the limit is meaningful in the output
        if i_term > self.i_max and self.i_gain != 0:
            i_term = self.i_max
            self.set_ierr = i_term / self.i_gain
        elif i_term < self.i_min and self.i_gain != 0:
            i_term = self.i_min
            self.set_ierr = i_term / self.i_gain
            
        # Calculate the derivative error
        self.set_derr = (self.set_perr - self.set_pstate) / dt
        self.set_pstate = self.set_perr
        
        # Calculate derivative contribution to command 
        d_term = self.d_gain * self.set_derr
        
        self.cmd = p_term + i_term + d_term

        return self.cmd