#!/usr/bin/env python
"""
@authors : oguzhankose - damlakonur

"""

import os
import numpy as np
import cv2
import traceback
import time

from LaneDetection import LaneDetection
from PreProcessImg import PreProcessImg

from zed_publisher import ZEDPublisher

from driver import DriverNode
from stanley import StanleyController

import rospy
import tf
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


past_coords = []


def img_callback(msg):

    start = time.time()

    img = CvBridge().imgmsg_to_cv2(msg, "bgr8").copy()

    seq = msg.header.seq

    try:
        #cv2.imwrite("inputs/input_{}.png".format(str(msg.header.seq)), img) 
        
        detector = LaneDetection([img.copy(), seq])
        
        pre_process = PreProcessImg([img.copy(), seq])
        
        
        # Pre Process
        pre_success, processedImg = pre_process.process_image()

        if pre_success == False:
            print("\n|| COULD NOT FOUND RED LANE FROM PREPROCESSING IMAGE ||\n")
            return False

            
        # Lane Detection 
        blended_img, out_img, lane_found, path = detector.process_image(processedImg)
        
        ###########################################     output contatenation
        out_img = cv2.copyMakeBorder(
                    out_img, 
                    (img.shape[0] - out_img.shape[0])/2, 
                    (img.shape[0] - out_img.shape[0])/2, 
                    (img.shape[1] - out_img.shape[1])/2, 
                    (img.shape[1] - out_img.shape[1])/2, 
                    cv2.BORDER_CONSTANT, 
                    value=(128,128,128)
                )
        comb = np.concatenate((blended_img, out_img), axis=0)
        ###########################################     output contatenation


        
        if lane_found:
            pose_list = []

            global past_coords
            past_coords.append([path[1][0], path[1][1], path[0][2]])

            for i in range(len(path)):

                pose = PoseStamped()
                pose.pose.position.x = path[i][0]
                pose.pose.position.y = path[i][1]
                pose.pose.position.z = path[i][2]

                quat = tf.transformations.quaternion_from_euler(0,0, np.deg2rad(path[i][2]))

                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]

                pose_list.append(pose)

                
            path_msg = Path()
            path_msg.poses = pose_list

            path_pub.publish(path_msg)
            
            org1 = (10, comb.shape[1]+35)
            org2 = (10, comb.shape[1]+55)
            org3 = (10, comb.shape[1]+75)
            org4 = (10, comb.shape[1]+15)

            # Using cv2.putText() method 
            cv2.putText(comb, "x:{0}, y:{1}, yaw:{2}".format(path[0][0], path[0][1], round(path[0][2], 1)), org1, cv2.FONT_HERSHEY_SIMPLEX,  
                    0.5, (0, 0, 255), 1, cv2.LINE_AA) 
            cv2.putText(comb, "x:{0}, y:{1}, yaw:{2}".format(path[1][0], path[1][1], round(path[1][2], 1)), org2, cv2.FONT_HERSHEY_SIMPLEX,  
                    0.5, (0, 255, 0), 1, cv2.LINE_AA) 
            cv2.putText(comb, "x:{0}, y:{1}, yaw:{2}".format(path[2][0], path[2][1], round(path[2][2], 1)), org3, cv2.FONT_HERSHEY_SIMPLEX,  
                    0.5, (255, 0, 0), 1, cv2.LINE_AA) 

            
            cv2.putText(comb, "Lane Found: {}".format(lane_found), org4, cv2.FONT_HERSHEY_SIMPLEX,  
                    0.5, (120, 180, 75), 1, cv2.LINE_AA) 
            

        else:
            past_coords = []
            cv2.putText(comb, "LANE IS NOT FOUND !!!".format(lane_found), (10, comb.shape[1]+15), cv2.FONT_HERSHEY_SIMPLEX,  
                    0.5, (120, 180, 75), 1, cv2.LINE_AA) 
                    
        cv2.imshow("Lane Detection", comb)
        cv2.waitKey(1)

        img_pub.publish(CvBridge().cv2_to_imgmsg(comb, "bgr8"))
        pr_img_pub.publish(CvBridge().cv2_to_imgmsg(processedImg))
        blended_img_pub.publish(CvBridge().cv2_to_imgmsg(blended_img, "bgr8"))
        tf_img_pub.publish(CvBridge().cv2_to_imgmsg(out_img, "bgr8"))
        
        
        #cv2.imwrite("outputs/output_{}.png".format(str(msg.header.seq)), comb) 
        
        

        
    except Exception as e:

        print("ERROR!")
        print(traceback.format_exc())

        img_pub.publish(CvBridge().cv2_to_imgmsg(img.copy(), "bgr8"))

        #quit()
        
        return False


    t = time.time() - start
    f = 1/t
    print("time : {0}, frequency: {1}".format(t, f))






if __name__ == "__main__": 

    rospy.init_node("lane_detection")

            
    rospy.Subscriber("/zed/zed_node/right/image_rect_color", Image, img_callback, queue_size=1, buff_size=52428800)

    img_pub = rospy.Publisher("/lane_detection/output", Image, queue_size=10)
    pr_img_pub = rospy.Publisher("/lane_detection/processed", Image, queue_size=10)
    blended_img_pub = rospy.Publisher("/lane_detection/blended", Image, queue_size=10)
    tf_img_pub = rospy.Publisher("/lane_detection/tfimg", Image, queue_size=10)


    path_pub = rospy.Publisher("/lane_detection/path", Path, queue_size=10)

  
    

    """
    os.chdir("/home/nvidia/")
    os.system("rm -rf inputs/")
    os.system("mkdir inputs")

    os.system("rm -rf outputs/")
    os.system("mkdir outputs")
    """

    rospy.spin()
    
    

