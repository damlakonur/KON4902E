"""
@author : damlakonur
Implementation of Discrete Pid Controller Structure
"""
import time
import math
import numpy as np



class zPIDController:

    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0):
        self.Ts = 0.05          # Sampling Time
        # Controller Coefficients
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.e_k1 = 0.0
        self.IntegralSum = 0.0
        self.e_k = 0.0


    def output(self, goal, current=0):

        self.e_k = goal - current
        self.IntegralSum += self.Integral(self.e_k, self.e_k1)
        
        derivative = self.Derivative(self.e_k, self.e_k1)

        output = self.Kp * self.e_k + self.IntegralSum*self.Ki + derivative*self.Kd
        self.e_k1 = self.e_k
        return output

    def Derivative(self, e_k, e_k1):

        return ((self.e_k - self.e_k1) / self.Ts)

    #Trepezoidal Integral

    def Integral(self, e_k, e_k1):

        return ((self.e_k + self.e_k1)*(self.Ts / 2))