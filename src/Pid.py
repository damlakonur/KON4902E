"""
@author : damlakonur
Implementation of  Pid Controller Structure
"""
import time
import math
import numpy as np



class PIDController:

    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.past_error = 0.0
        self.prev_time = time.time()

    def output(self, goal, current=0):
        cur_time = time.time()
        dt = cur_time - self.prev_time

        lat_error = goal - current

        proportional = self.Kp * lat_error
        self.integral += self.Ki * (lat_error + self.past_error) * dt
        derivative = self.Kd * (lat_error - self.past_error) / dt

        self.integral = min(self.integral, 10)

        output = proportional + self.integral + derivative

        self.past_error = lat_error
        self.prev_time = cur_time

        return output

class I_PIDController:
    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0,alpha = 1,filt_size = 5,saturation=10):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.saturation=saturation
        self.alpha = alpha
        self.filt_size = filt_size
        self.der_filt=FIR_filt(1.0/self.filt_size*np.ones(self.filt_size))
        self.der2_filt=FIR_filt(1.0/self.filt_size*np.ones(self.filt_size))
        self.integral = 0.0
        self.past_error = 0.0
        self.past_der = 0.0
        self.prev_time = time.time()
        self.prev_output = 0.0


    def output(self, goal, current=0):
        cur_time = time.time()
        dt = cur_time - self.prev_time

        lat_error = goal - current

        proportional = self.Kp/self.alpha * lat_error
        self.integral += self.Ki * lat_error + self.past_error * dt
        derivative = self.Kd * (lat_error - self.past_error) / dt
        self.der_filt.update(derivative)
        derivative=self.der_filt.output()

        d_derivative = 1/self.alpha * (derivative - self.past_der) / dt
        self.der2_filt.update(derivative)
        d_derivative=self.der2_filt.output()

        self.integral = min(self.integral, self.saturation)
        self.integral = max(self.integral, -self.saturation)

        d_output = proportional + self.Ki*self.integral + derivative + d_derivative
        output=self.prev_output+d_output*dt

        output = min(output, self.saturation)
        output = max(output, -self.saturation)

        self.prev_output=output

        self.past_error = lat_error
        self.prev_time = cur_time
        print("prop "+str(proportional)+" int "+str(self.integral)+" der "+str(derivative)+" d_der "+str(derivative)+" d_out "+str(d_output)+" prev_out "+str(self.prev_output))
        return output
class FIR_filt:
    def __init__(self,filt_coeff = [0.2,0.2,0.2,0.2,0.2]):
        self.filt_coeff=np.array(filt_coeff)
        self.window=np.zeros(len(self.filt_coeff))
    def update(self,x):
        for i in range(len(self.window)-1):
            self.window[len(self.window)-i-1]=self.window[len(self.window)-i-2]
        self.window[0]=x
    def output(self):
        return np.dot(self.window,self.filt_coeff)


