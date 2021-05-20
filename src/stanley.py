
"""
@author : damlakonur
Implementation of Stanley Controller Structure
"""
import rospy
import os
import traceback
import time
import math
import numpy as np


class StanleyController():
    def __init__(self, k_e=0.0, vel=0.0):
        # Controller Coefficients
        self.k_e = k_e
        self.k_s = 5.0
        self.h_e = 0.019

        self.vel = vel
        self._cross_track_error = 0.0
        self.steering = 0.0
        self.curv = 0.0
        self.prev_steering = 0.0
        self.max_steering_rad = 0.6     # Saturation Limit
        self.min_steering_rad = -0.6    # Saturation Limit
        self.dt = 0.05                  # Sampling Time



    def output(self, goal, curvature, current=0):

        self.curv = curvature
        self._cross_track_error = goal - current
        
        _cross_track_steering = self.cross_track_steering(self._cross_track_error, self.k_e, self.vel, self.k_s)
        _heading_error = self.heading_error(curvature = self.curv)


        self.steering = -_heading_error + _cross_track_steering


        # Saturation
        
        if(self.max_steering_rad < self.steering):
            self.steering = self.max_steering_rad

        elif(self.steering < self.min_steering_rad):
            self.steering = self.min_steering_rad
        

        # low pass filter
        low_pass_gain = 0.9

        self.steering = (1 - low_pass_gain)*self.prev_steering + self.steering*low_pass_gain
        self.prev_steering = self.steering
        print("headiing_error :", _heading_error)
        print("steer : ", self.steering)
        return self.steering

    # Calculate Cross Track Error
    def cross_track_steering(self, _cross_track_error, k_e, vel, k_s):

        
        return self.k_e*(math.atan((self.k_e * self._cross_track_error) / (self.k_s + self.vel)))

    # Calculate Heading Error 
    def heading_error(self, curvature, prev_yaw=0):

        _heading_error = self.curv - prev_yaw

        print(self.h_e*_heading_error)

        return self.h_e*_heading_error
    

