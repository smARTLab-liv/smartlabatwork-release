#!/usr/bin/env python
import rospy
import actionlib

from sensor_msgs.msg import Image

from slaw_msgs.msg import *

#from slaw_manipulation.grip_and_place import GripAndPlace

from cv_bridge import CvBridge, CvBridgeError

import thread
import sys
import cv
import cv2
import numpy as np
import random
from math import sin, cos, pi
import math
from vision_helpers import process_rgb, crop_x, crop_y, filter_contours


CROSS_OFF_X = 10
CROSS_OFF_Y = -54



class DetectObjects():
    def __init__(self):
        self.bridge = CvBridge()

        self.fix_count = 0

        self.angle = None

        # init camera
        rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.cb_image)

        
    def cb_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv(msg, "bgr8")
            frame = np.array(frame, dtype=np.uint8)
            self.process_image(frame)
            key = cv.WaitKey(5)
            #print key
            if key == ord('r'):
                # restart
                rospy.sleep(3)

        except CvBridgeError, e:
            print e

            
    def process_image(self, frame):
        frame = frame[crop_y[0]:crop_y[1], crop_x[0]:crop_x[1]]

        size = frame.shape
        size = (size[1]-1, size[0]-1)

        frame_dilate = process_rgb(frame)

        # contours
        contours, hierarchy = cv2.findContours(frame_dilate, 
                                               cv2.RETR_EXTERNAL, 
                                               cv2.CHAIN_APPROX_SIMPLE)

        contours, approx = filter_contours(contours)

        cross_w = size[0]/2 + CROSS_OFF_X
        cross_h = size[1]/2 + CROSS_OFF_Y
        
            #crosshair
        cv2.line(frame,
                 (cross_w, 0),
                 (cross_w, size[1]),
                 (255,255,0))

        cv2.line(frame,
                 (0, cross_h),
                 (size[0], cross_h),
                 (255,255,0))


        largest = None
        max_area = 0
        smallest_x_dist = size[0]
        snd_smallest_x_dist = size[0]

        #cen1 = None
        #cen2 = None#

        #print cen1[0] - size[0]/2
        #print cen1[1] - size[1]/2
        
        #print  np.linalg.norm((np.array(cen2) - np.array(cen1)))
        #print (np.array(cen2) + np.array(cen1)) /2.

        for c in contours:
            ellipse = cv2.fitEllipse(c)
            center = np.int32(ellipse[0])
            
            cv2.circle(frame, tuple(center), 3, 255, 2)
            
            xdist = center[0] - cross_w
            ydist = center[1] - cross_h

            print xdist, ydist
            
                    
        # show camera image with annotations
        cv2.imshow("camera contours", frame)

                    
if __name__ == "__main__":
    rospy.init_node("visual_servoing")
    vs = DetectObjects()
    rospy.sleep(1)
    

    
    try:
        rospy.spin()
    except Keyboardinterrupt:
        pass

    # to be save send 0-twist
    #vs.send_twist(0, 0)
    cv.DestroyAllWindows()
    
