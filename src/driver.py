import rospy
import os
import traceback
import time
import math
import numpy as np
import csv
import cv2

from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
from zPid import zPIDController
from geometry_msgs.msg import Twist
from stanley  import StanleyController



class DriverNode():

    def __init__(self, cmd_type="path"):

        self.type = cmd_type
        self.speed = 0.8            # Vehicle initial speed
        self.mode = "MANUAL"
        self.modee = 0
        self.ref_vel = 0.0
        self.curv = 0.0
        self.k_v = 0.5
        self.prev_speed = 0.0

        self.start = time.time()

        self.joy_steer = 0
        self.mode = 0
        self.prevTime  = 0.0
        self.target_y = 0.0
        self.steering_angle = 0.0

        # This code used to save experiment data
        
        self.x_data, self.y_data, self.y_data2, self.input, self.curvature, self.vel = [], [], [], [], [], []
        
        
        rospy.Subscriber("/lane_detection/path", Path, self.pose_callback, queue_size=10)
        rospy.Subscriber("/joy", Joy, self.joy_callback, queue_size=10)
        self.ack_msg = AckermannDriveStamped()
        self.ack_pub = rospy.Publisher("/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=10)

    # Joystick Conf.

    def joy_callback(self, joy):
    
        if(joy.axes[5] == -1):
            self.mode = "AUTO"
            self.modee = 1
        
        elif(joy.buttons[4]):
            self.mode = "MANUAL"
            self.modee = 0

        else:
            self.mode = "DISABLED"
            self.modee = 0
        
    def pose_callback(self, msg):

        
        # Lateral Error
        self.target_y = msg.poses[0].pose.position.y -18.0
        # The curvature is found by calculating the slope between two points on the fitted polynomial
        self.curv = math.radians(abs(msg.poses[1].pose.position.z))

        # Velocity Control is performing by basic formula that changes wrt road curvature
        self.velocity = self.speed*(1.5 - self.k_v*self.curv)
        # Low Pass Filter is applied to velocity
        low_pass_gain = 0.9
        self.velocity = (1 - low_pass_gain)*self.prev_speed + self.velocity*low_pass_gain
        self.prev_speed = self.velocity
        # Calculated Velocity is applied to the vehicle
        self.ack_msg.drive.speed = self.velocity
        # To use Stanley, uncomment following lines
        """
        self.Stanley = StanleyController(k_e = 0.26, vel=self.velocity)
        self.steering_angle = self.Stanley.output(goal=self.target_y, curvature=self.curv)
        self.ack_msg.drive.steering_angle = -self.steering_angle
        """
        # To use P Controller, uncomment following lines
        """
        self.Pid = zPIDController(Kp = 0.01, Ki = 0.0, Kd = 0.0)
        self.steering_angle = self.Pid.output(goal=self.target_y)
        self.ack_msg.drive.steering_angle = -self.steering_angle
        """
        # To use PD Controller, uncomment following lines
        """
        self.Pid = zPIDController(Kp = 0.01, Ki = 0.0, Kd = 0.02)
        self.steering_angle = self.Pid.output(goal=self.target_y)
        self.ack_msg.drive.steering_angle = -self.steering_angle
        """

        if(self.mode == "AUTO"):
            self.ack_pub.publish(self.ack_msg)
        else:
            pass


    # This function written to generate velocity ref but it wasn't used
    def curv_type(self, curvature):

        
        c_normal = 0.20
        c_sharp = 1.1

        if ((0.0 < self.curv) and (self.curv < c_normal)):
            return 0.0
        elif ((c_normal < self.curv) and (self.curv < c_sharp)):
            return 0.2
        elif(self.curv > c_sharp):
            return 0.6
        

        

if __name__ == "__main__":
    
    rospy.init_node('driver')

    sender = DriverNode(cmd_type="path")


    vis = DriverNode()

    # Save data with constant sample time 
    r = rospy.Rate(20)

    time.sleep(1)
    while not rospy.is_shutdown():

        with open('record10.csv', 'wb+') as ioFile:
            
            try:
                vis.x_data.append(time.time()-vis.start)
                vis.input.append(vis.modee)
                vis.y_data.append(vis.target_y)
                vis.y_data2.append(vis.steering_angle)
                vis.curvature.append(vis.curv)
                vis.vel.append(vis.velocity)
                data = np.array([vis.y_data, vis.y_data2, vis.x_data, vis.input, vis.curvature, vis.vel])
                data = data.T
                np.savetxt(ioFile, data, fmt=['%.2f', '%.2f', '%.2f', '%d', '%.2f', '%.2f'], delimiter="\t")
            except Exception as e:
                print("HATA")
                print(e)
        

        r.sleep()
    
    


   



