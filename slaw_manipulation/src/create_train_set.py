#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import cv
import cv2
import numpy as np

import datetime
import sys, getopt
import os

class TrainingSetCreator():
    def __init__(self):
        self.bridge = CvBridge()

        # init camera
        rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.process_color_image)
        rospy.Subscriber("/camera/depth_registered/hw_registered/image_rect_raw", Image, self.process_depth_image)
        
        self.depth_image = None
        self.color_image = None
        self.orientation = None
        self.objName = None
        
        self.load_arguments()
        self.folder = "./trainingSet/" + self.orientation + "/" + self.objName 
        self.depthDir = self.folder + "/depth/"
        self.colorDir = self.folder + "/color/"
        self.rgbDir = self.folder + "/rgb/"

        if not os.path.exists(self.depthDir):
            os.makedirs(self.depthDir)
        if not os.path.exists(self.colorDir):
            os.makedirs(self.colorDir)
        if not os.path.exists(self.rgbDir):
            os.makedirs(self.rgbDir)
        
        self.processInfo()
        self.start_control_loop()
    
    #read the arguments for objectName and Orientation
    def load_arguments(self):
        try:
            opts, args = getopt.getopt(sys.argv[1:],'o:n:h',['orientation=','objName=','help'])
        except getopt.GetoptError as e:
            print (str(e))
            self.argError()
            sys.exit(2)
        
        for o, a in opts:
            if o in ('-o', '--orientation'):
                if a == 'v':
                    self.orientation = 'vertical'
                elif a == 'h':
                    self.orientation = 'horizontal'
                else:
                    print "WRONG ORIENTATION!! Values can only be: v or h! v = vertical and h = horizontal"
                    exit(2)
            if o in ('-n', '--objName'):
                self.objName = a
            if o in ('-h', '--help'):
                self.argError()
                exit(2)
        
        if self.objName is None or self.orientation is None:
            self.argError()
            exit(2)
    

    def processInfo(self):
        print "Program started"
        print "Program info:"
        print "\nSaving images in %s" % self.folder


    def argError(self):
        print "example: python2.7 create_train_set.py --orientation h --objName m20"
        print "\nOptions:\n"
        print "-o --orientation             Orientation of object values:"
        print "                             h->horizontal and v->vertical"
        print "-n --objName                 Name of object"
        print "-h --help                    help"
        
 
    def start_control_loop(self):
        rospy.init_node("TrainingSetCreator")
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.show_and_save_images()
            r.sleep()
   

    def show_and_save_images(self):
        if self.depth_image is not None and self.color_image is not None:
            cv2.imshow("depth_image", self.depth_image)
            cv2.imshow("color_image", self.color_image)
            key = cv2.waitKey(3)
            #if key is pressed, save images
            if key is not -1:
                dateStr = datetime.datetime.utcnow().strftime('%Y%m%d_%H%M%S%f')[:-3]
                rgbImg = self.rgbDir + dateStr
                
                depthImg = self.depthDir + dateStr
                colorImg = self.colorDir + dateStr + ".png"
                print "Saving color image to %s" % colorImg
                cv2.imwrite(colorImg, self.color_image)
                print "Saving depth numpy array to %s" % depthImg
                #cv2.imwrite(depthImg, self.depth_image)
                np.savez_compressed(depthImg, self.depth_image)
                print "Saving rgb numpy array to %s" % rgbImg
                np.savez_compressed(rgbImg, self.color_image)

    
    def process_depth_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv(msg, "32FC1")
            frame = np.array(frame, dtype=np.float32)
            self.depth_image = frame 
        except CvBridgeError as e:
            pass


    def process_color_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv(msg, "bgr8")
            frame = np.array(frame, dtype=np.uint8)
            self.color_image = frame 
        except CvBridgeError as e:
            pass

                    
if __name__ == "__main__":
    try:
        TrainingSetCreator()
    except KeyboardInterrupt:
        pass

    cv.DestroyAllWindows()
