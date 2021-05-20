#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
@authors : oguzhankose - damlakonur

"""

import numpy as np
import cv2
import os
import sys
import math
import matplotlib.pyplot as plt
import traceback


class PreProcessImg():

    def __init__(self, img):
        

        #   ###
        #   1 Take Image and Seq as Input
        # img[0] for img img[1] for img name
        self.orig_img = img[0]
        self.img_name = img[1]
        self.imshape = img[0].shape

    # HSV Filter for detecting red lane
    def hsv_red_lane(self, thresholdImg):
        # Set minimum and max HSV values to display
        lower = np.array([150, 20, 70])
        upper = np.array([179, 255, 255])


        # Create HSV Image and threshold into a range.
        hsv = cv2.cvtColor(self.orig_img.copy(), cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, lower, upper)
 
        mask[0:220] = 0
        mask[mask.shape[0]-20:mask.shape[0]] = 0

        mask = cv2.morphologyEx(mask.copy(), cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (26,26)))
        kernel = np.array([[0,1,0],[1,1,1],[0,1,0]], dtype="uint8")
        mask = cv2.morphologyEx(mask.copy(), cv2.MORPH_OPEN, kernel, iterations = 3)  
    
        try:
            contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        except:
            _, contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        hsv_list = [cv2.contourArea(cnt) for cnt in contours]
        max_hsv_index = hsv_list.index(max(hsv_list))

        if cv2.contourArea(contours[max_hsv_index]) > 400 and cv2.contourArea(contours[max_hsv_index]) < 5000:
            
            mask = cv2.drawContours(np.zeros_like(mask), contours, max_hsv_index, 255, -1)
        else:
            mask = np.zeros_like(mask)
            

        hsv_thresh = np.zeros_like(thresholdImg)
        hsv_thresh = cv2.bitwise_not(hsv_thresh.copy(), thresholdImg.copy(), mask=mask)


        return hsv_thresh
        

    def process_image(self):
        # 1 #
        # Take input image from zed

        # 2 #
        # Gray Scale 
        grayImg = cv2.cvtColor(self.orig_img, cv2.COLOR_BGR2GRAY)

        # 3 #
        # Mask
        vertices = np.array([[(0,self.imshape[0]), (0, 210), (self.imshape[1], 210), (self.imshape[1],self.imshape[0])]], dtype=np.int32)
        mask = np.zeros_like(grayImg)  
        cv2.fillPoly(mask, vertices, 255)
        maskedImg = cv2.bitwise_and(grayImg, mask)

        # 4 #
        # Apply HSV Filter to masked image
        try:
            hsv_thresh = self.hsv_red_lane(np.zeros_like(maskedImg))

            return True, hsv_thresh

        except Exception as e:
            hsv_thresh = maskedImg

            return False, hsv_thresh

        

        

        
       



        
        





        

                
        
