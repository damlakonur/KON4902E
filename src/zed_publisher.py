#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import time


class ZEDPublisher():

    def __init__(self):

        self.img_pub = rospy.Publisher("/zed_camera/frame", Image, queue_size=52428800)

        self.cap = cv2.VideoCapture(0)

        self.cap.set(3, 1344)
        self.cap.set(4, 376)




    def get_frame(self):

        

        ret, frame = self.cap.read()

        img = np.split(frame.copy(), 2, axis=1)[1]

        self.img_pub.publish(CvBridge().cv2_to_imgmsg(img, "bgr8"))

        


if __name__ == "__main__": 

    rospy.init_node("zed_publisher")


    zed = ZEDPublisher()

    rate = rospy.Rate(24) 

    while not rospy.is_shutdown():

        start = time.time()

        zed.get_frame()

        rate.sleep()

        t = time.time() - start
        f = 1/t
        #print("time : {0}, frequency: {1}".format(t, f))

                    
        
        




    

