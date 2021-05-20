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
from numpy.polynomial.polynomial import polyfit, polyval, polyder
import matplotlib.pyplot as plt 
import traceback
import warnings
with warnings.catch_warnings():
    warnings.filterwarnings('ignore', r'All-NaN (slice|axis) encountered')



class LaneDetection():

    def __init__(self, img):
        
        # img[0] for img img[1] for img name
        self.orig_img = img[0]
        self.img_name = img[1]
        
        self.tf_img = np.zeros((250, 314), dtype="uint8")

        self.imshape = img[0].shape

        self.st_point = 200
        self.md_point = 165
        self.fi_point = 100
       

    def get_poly(self, contours):

        poly_list = []
        draw_list = []
        # Sorting contours wrt their area
        contours = sorted(contours, key = cv2.contourArea, reverse = True)

        for cnt in contours[:3]:
            # Elimination by area
            if cv2.contourArea(cnt) < 10:
                continue
            
            py = cnt[:,0,0]
            px = cnt[:,0,1]

            poly = polyfit(px, py, 2)

            draw_x = np.array(range(self.fi_point, self.st_point), dtype=np.int32)
            draw_y = np.array(polyval(draw_x, poly), dtype=np.int32)   # evaluate the polynomial

            draw_points = np.array(zip(draw_y, draw_x), dtype=np.int32)

            poly_list.append(poly)
            draw_list.append(draw_points)
            perimeter = cv2.arcLength(cnt,True)

            
        if len(poly_list) >= 1:    
            return poly_list, draw_list, True
        else:
            return [], [], False


    
    def tf_image(self, img, mode):

        src = np.array([[355, 204], [253, 205], [34, 335], [522, 326]], dtype=np.float32)
        dst = np.array([[70+122,0], [0+122, 0], [0+122,200], [70+122,200]], dtype=np.float32)

        if(mode == 'birdeye'):
            persp_tf = cv2.getPerspectiveTransform(src, dst)

        elif(mode == 'reverse'):
            persp_tf = cv2.getPerspectiveTransform(dst, src)

        else:
            print('Wrong Image Transform Mode')
            return False


        img = cv2.warpPerspective(img, persp_tf, (314, 250))       ### kalibrasyonndegerleri degistir class dan

        return img


    
    def get_target_points(self, src, poly_list, draw_points_list):

        pointsImg = src
        start = [None] * len(poly_list)
        median = [None] * len(poly_list)
        finish = [None] * len(poly_list)
        orgin_dist = [None] * len(poly_list)
        for i in range(len(poly_list)):

            draw_points = draw_points_list[i]

            t_p_x = draw_points[:,0]
            t_p_y = draw_points[:,1]

            start[i] = (int(polyval(self.st_point, poly_list[i])), self.st_point)
            finish[i] = (int(polyval(self.fi_point, poly_list[i])), self.fi_point)
            median[i] = (int(polyval(self.md_point, poly_list[i])), self.md_point)

            cv2.circle(pointsImg, start[i], 5, (255,0,0), -1)
            cv2.circle(pointsImg, median[i], 5, (0,255,0), -1)
            cv2.circle(pointsImg, finish[i], 5, (0,0,255), -1)

            orgin_dist[i] = abs((pointsImg.shape[1]/2) - int(polyval(self.st_point, poly_list[i])))

        
        goal_list = [None, None, None]
        goal_list[0] = tuple(map(int, tuple(np.array(start).mean(axis=0))))
        goal_list[2] = tuple(map(int, tuple(np.array(finish).mean(axis=0))))
        goal_list[1] = tuple(map(int, tuple(np.array(median).mean(axis=0))))

        """
        cv2.circle(pointsImg, goal_list[0], 5, (255, 0, 0), -1)
        cv2.circle(pointsImg, goal_list[1], 5, (0, 255, 0), -1)
        cv2.circle(pointsImg, goal_list[2], 5, (0, 0, 255), -1)


        cv2.line(pointsImg, goal_list[0], goal_list[1], (255,0,0), 8) # plot line
        cv2.line(pointsImg, goal_list[1], goal_list[2], (100,100,0), 8) # plot line
        """


        xmax = float(self.tf_img.shape[0])
        ymax = float(self.tf_img.shape[1])

        path = []

        x1 = xmax - goal_list[0][1]
        y1 = goal_list[0][0] - (ymax/2)
        sl1 = math.degrees(math.atan(y1 / x1))
        path.append([x1, y1, sl1])

        x2 = xmax - goal_list[1][1]
        y2 = goal_list[1][0] - (ymax/2)

        print(x1, x2, y1, y2)
        if x1==x2:
            sl2 = 0.0
        else:
            sl2 = math.degrees(math.atan((y2-y1) / (x2-x1)))
        path.append([x2, y2, sl2])

        x3 = xmax - goal_list[2][1]
        y3 = goal_list[2][0] - (ymax/2)
        sl3 = sl2
        path.append([x3, y3, sl3])

        return pointsImg, path, goal_list


    # After Pre Processing
    def process_image(self, pr_img):


        lane_found = False

        # 1 Bird Eye Transformation
        tf_laneContours = self.tf_image(img=pr_img.copy(), mode='birdeye')
        # 2 Find Contours
        try:
            contours, _ = cv2.findContours(tf_laneContours.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        except:
            _, contours, _ = cv2.findContours(tf_laneContours.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

       
        # 3 Fit Poly
        poly_list, draw_list, lane_found = self.get_poly(contours)

        
        # LANE FOUND
        if(lane_found):
            
            tfimg = cv2.cvtColor(tf_laneContours.copy(), cv2.COLOR_GRAY2BGR)
            output_tf, path, goal_list = self.get_target_points(tfimg.copy(), poly_list, draw_list)
 
            for draw in draw_list:
                color = list(np.random.random(size=3) * 256)
                cv2.polylines(output_tf, [draw], False, color=color, thickness = 4)  # args: image, points, closed, color


            # plot blended img
            blendedImg_out = self.orig_img.copy()
            ind = np.where(pr_img==[255])
            blendedImg_out[ind] = (255,0,0)

            return blendedImg_out, output_tf, len(poly_list), path


        # NO LANE FOUND
        else:
            tfimg = np.zeros_like(cv2.cvtColor(tf_laneContours.copy(), cv2.COLOR_GRAY2BGR))
            pr_img = cv2.cvtColor(pr_img.copy(), cv2.COLOR_GRAY2BGR)
            
            print("None of the Lanes could be found!!")
            return pr_img, tfimg, 0, []


        

        

                
        
