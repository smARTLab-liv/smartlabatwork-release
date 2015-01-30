#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# import thread
import cv
import cv2
import numpy as np
# import tf
from slaw_srvs.srv import CheckRV20, CheckRV20Response
from vision_helpers import filter_low_high

crop_y = [5, 105]
crop_x = [75, 150]

MIN = 0.12
MAX = 0.18

MIN_SEEN_CIRCLES = 4

NUM_FRAMES = 40
VISUALIZE = True

class DetectRV20(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_frame = None
        rospy.Subscriber("/softkinetic_camera/depth/image_raw", Image, self.cb_depth)

        while self.depth_frame is None:
            if rospy.is_shutdown():
                return
            rospy.logwarn('RV20 waiting for first depth frame')
            rospy.sleep(0.1)
        rospy.Service("/vision/check_rv20", CheckRV20, self.cb_start_check)

    def cb_start_check(self, req):
        res = CheckRV20Response()
        rate = rospy.Rate(20)

        R20 = 0
        V20 = 0
        rospy.sleep(1.0)
        for i in xrange(NUM_FRAMES):
            while self.depth_frame is None:
                if rospy.is_shutdown():
                    return res
                rate.sleep()
            depth_frame = self.bridge.imgmsg_to_cv(self.depth_frame, "32FC1")

            self.depth_frame = None
            depth_frame = np.array(depth_frame, dtype=np.float32)
            tmp_img = np.copy(depth_frame)
            cv2.rectangle(tmp_img, (crop_x[0], crop_y[0]), (crop_x[1], crop_y[1]), (255, 255, 255), 2)
            cv2.imshow('full image', tmp_img)
            depth_frame = depth_frame[crop_y[0]:crop_y[1], crop_x[0]:crop_x[1]]
            num_circles = self.process_image(depth_frame)
            if num_circles > 0:
                R20 += 1
            else:
                V20 += 1
        print R20, V20, 'R, V'
        print 'MIN_SEEN_CIRCLES' + str(MIN_SEEN_CIRCLES)
        if R20 > MIN_SEEN_CIRCLES:
            res.result = 'R20'
        else:
            res.result = 'V20'
        return res

    def cb_depth(self, msg):
        self.depth_frame = msg

    def process_image(self, depth_frame):

        filtered = self.process_depth(depth_frame)

        circles = cv2.HoughCircles(filtered, cv.CV_HOUGH_GRADIENT, 1, 20,
                                   param1=50, param2=30, minRadius=0, maxRadius=0)
        count = 0
        if circles is not None:
            circles = np.uint16(np.around(circles))
            if VISUALIZE:
                for i in circles[0, :]:
                    # draw the outer circle
                    cv2.circle(depth_frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
                    # draw the center of the circle
                    cv2.circle(depth_frame, (i[0], i[1]), 2, (0, 0, 255), 3)
            count = len(circles[0, :])
        if VISUALIZE:
            cv2.imshow('depth', depth_frame)
            cv2.waitKey(5)
        return count


    def process_depth(self, depth_img):
        # # Filter NaN
        idx = np.isnan(depth_img)
        depth_img[idx] = 0
        # # Convert to UINT8 image
        #print np.min(depth_img), np.max(depth_img), np.mean(depth_img)
        depth_img = filter_low_high(depth_img, MIN, MAX)
        #print np.min(depth_img), np.max(depth_img), np.mean(depth_img)

        depth_img = depth_img / (MAX) * 255
        depth_img = np.uint8(depth_img)
        depth_img = cv2.medianBlur(depth_img, 9)


        frame_filter = cv2.adaptiveThreshold(depth_img,
                                             255,
                                             cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                             # cv2.ADAPTIVE_THRESH_MEAN_C,
                                             cv2.THRESH_BINARY,

                                             11,  # neighbourhood
                                             2)
        cv2.bitwise_not(frame_filter, frame_filter)
        kernel = np.ones((1, 1), 'uint8')
        frame_filter = cv2.dilate(frame_filter, kernel)

        return frame_filter


if __name__ == "__main__":
    rospy.init_node("RV20_Check")
    detect = DetectRV20()
    rospy.spin()

