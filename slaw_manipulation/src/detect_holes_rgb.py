#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# import thread
import cv
import cv2
import numpy as np
import tf

from math import sin, cos
import math
from slaw_msgs.msg import PoseStampedLabeled, PoseStampedLabeledArray
from j48 import j48_holes
import vision_constants

from vision_helpers import filter_contours, merge_contours, get_holes, pixel_to_m, \
    get_darkest_hole, merge_contours_holes, find_m20_head, rotate_point, \
    m_to_pixel  # single_full2crop_coordinates

FULL_VISUALIZE = False
NO_VISUALIZATION = True


class DetectHolesRGB():
    def __init__(self, training=False):
        self.bridge = CvBridge()

        self.cur_rgb = None
        self.training = training

        if not training:
            # init camera
            rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.cb_rgb)

            self.img_pub = rospy.Publisher("/vision/image_holes", Image)
            self.pose_pub = rospy.Publisher("/vision/detected_holes", PoseStampedLabeledArray)
            rate = rospy.Rate(30)

            while not rospy.is_shutdown():
                if self.cur_rgb is None:
                    rate.sleep()
                    continue
                color = self.bridge.imgmsg_to_cv(self.cur_rgb, "bgr8")
                color = np.array(color, dtype=np.uint8)
                self.cur_rgb = None
                self.cb_image(color)
                rate.sleep()

    def cb_rgb(self, msg):
        self.cur_rgb = msg

    def process_rgb(self, rgb_img):
        frame_gray = cv2.cvtColor(rgb_img, cv.CV_RGB2GRAY)
        # gray_blurred = cv2.GaussianBlur(frame_gray, (9, 9), 0)
        gray_blurred = cv2.medianBlur(frame_gray, 5)
        # gray_blurred = cv2.bilateralFilter(frame_gray, 8, 16, 4)
        # cv2.imshow("gray_blurred", gray_blurred)


        gray_filter = cv2.adaptiveThreshold(gray_blurred,
                                            255.0,
                                            # cv.CV_ADAPTIVE_THRESH_GAUSSIAN_C,
                                            cv.CV_ADAPTIVE_THRESH_MEAN_C,
                                            cv.CV_THRESH_BINARY,
                                            9,  # neighbourhood
                                            9)
        cv2.bitwise_not(gray_filter, gray_filter)
        kernel = np.ones((3, 3), 'uint8')
        # gray_erode = gray_filter
        gray_erode = cv2.erode(gray_filter, kernel)

        kernel2 = np.ones((5, 5), 'uint8')

        gray_erode = cv2.dilate(gray_erode, kernel2)
        gray_erode = cv2.erode(gray_erode, kernel)

        size = rgb_img.shape
        size = (size[1] - 1, size[0] - 1)

        cv2.rectangle(gray_erode, (0, 0), size,
                      0,  # color
                      20,  # thickness
                      8,  # line-type ???
                      0)  # random shit

        return gray_erode


    def cb_image(self, color_frame):
        try:
            # # Crop image
            color_frame = color_frame[vision_constants.crop_y[0]:vision_constants.crop_y[1],
                          vision_constants.crop_x[0]:vision_constants.crop_x[1]]

            self.process_image(color_frame)
            if not NO_VISUALIZATION:
                cv.WaitKey(5)

        except CvBridgeError, e:
            print e

    def process_image(self, color_img):
        # make copys
        color_img_display = color_img.copy()
        frame_gray = cv2.cvtColor(color_img, cv.CV_RGB2GRAY)

        processed_rgb = self.process_rgb(color_img)

        if FULL_VISUALIZE and not NO_VISUALIZATION:
            cv2.imshow("processed_rgb", processed_rgb)

        contours_rgb, hierarchy = cv2.findContours(processed_rgb,
                                                   cv2.RETR_EXTERNAL,
                                                   cv2.CHAIN_APPROX_SIMPLE)



        # # Filter contours (min max area and min max axis length)
        contours_rgb = merge_contours(contours_rgb)
        contours_rgb, approx_rgb = filter_contours(contours_rgb)
        contours_rgb = merge_contours_holes(contours_rgb)

        size = color_img.shape
        size = (size[1] - 1, size[0] - 1)

        cross_w = size[0] / 2 + vision_constants.CROSS_OFF_X
        cross_h = size[1] / 2 + vision_constants.CROSS_OFF_Y

        cv2.line(color_img_display,
                 (cross_w, 0),
                 (cross_w, size[1]),
                 (255, 255, 0))

        cv2.line(color_img_display,
                 (0, cross_h),
                 (size[0], cross_h),
                 (255, 255, 0))

        msg_array = PoseStampedLabeledArray()

        if not self.training:
            time = rospy.Time.now()
            msg_array.header.stamp = time
            msg_array.header.frame_id = '/arm_base_link'
            msg_array.detection_type = 'rgb'

        # ## RGB Pipeline
        #cv2.drawContours(color_img_display, contours_rgb, -1, (255,0,0), 3)


        global_mask_rgb = np.zeros(color_img.shape, dtype=np.uint8)

        holes_rgb = get_holes(contours_rgb, frame_gray, global_mask=global_mask_rgb)

        ## walk through holes
        if not self.training:
            for i, o in enumerate(holes_rgb):
                hole_label = "unknown"
                center = o['center']
                angle = o['angle']
                features = o['features']

                cv2.drawContours(color_img_display, [o['contour']], -1, (255, 0, 0), 3)

                cv2.circle(color_img_display, tuple(center), 3, (0, 0, 255), 2)
                r = 30
                #res = classifier_r.find_label(features)
                #print res
                #hole_label = res[1]

                hole_label = j48_holes(features)

                cv2.putText(color_img_display, hole_label,
                            (center[0] - len(hole_label) * 7, center[1] + 15 + int(features[2] / 2)),
                            cv2.FONT_HERSHEY_PLAIN,
                            1.5, (0, 255, 0), thickness=2)

                if hole_label == 'M20_100_horizontal':
                    string_res, res = find_m20_head(o['contour'], center, -angle)
                    #print res
                    if string_res == 'Down':
                        #print 'Down', angle
                        #angle = math.pi - angle
                        vec = [[0, -1]]
                        rot_vec = np.array(rotate_point(vec, (0, 0), angle))
                        #vec = np.array([sin(angle),-cos(angle)])
                        vec = np.int32(rot_vec * 1 / 2. * features[2])
                        cv2.circle(color_img_display, tuple(center + vec), 3, (0, 0, 255), 2)
                        angle += math.pi
                        #print vec
                    if string_res == 'Top':
                        vec = [[0, 1]]
                        rot_vec = np.array(rotate_point(vec, (0, 0), angle))
                        #vec = np.array([sin(angle),-cos(angle)])
                        vec = np.int32(rot_vec * 1 / 2. * features[2])
                        cv2.circle(color_img_display, tuple(center + vec), 3, (0, 0, 255), 2)
                        #print 'Top', angle
                        #print vec
                    center += m_to_pixel(0.015) * rot_vec
                    #cv2.drawContours(color_img_display, [res], -1, (255,0,0), 3)

                cv2.line(color_img_display,
                         tuple(np.int32(center) + np.int32([r * cos(angle), r * sin(angle)])),
                         tuple(np.int32(center) - np.int32([r * cos(angle), r * sin(angle)])),
                         (0, 0, 255),
                         2)
                ## create message
                xdist = center[0] - cross_w
                ydist = center[1] - cross_h

                x_m = pixel_to_m(xdist)
                y_m = pixel_to_m(ydist)

                msg = PoseStampedLabeled()
                msg.pose.header.frame_id = '/arm_base_link'
                msg.pose.header.stamp = time

                #double check
                msg.pose.pose.position.x = y_m
                msg.pose.pose.position.y = x_m

                quat = tf.transformations.quaternion_from_euler(0, 0, angle)

                msg.pose.pose.orientation.x = quat[0]
                msg.pose.pose.orientation.y = quat[1]
                msg.pose.pose.orientation.z = quat[2]
                msg.pose.pose.orientation.w = quat[3]
                #print xdist, ydist
                #print self.angle

                msg.label = hole_label
                msg.detection_type = 'rgb'
                msg_array.poses.append(msg)

        if self.training:
            best_rgb = get_darkest_hole(holes_rgb, frame_gray, size)
            cv2.drawContours(color_img_display, [best_rgb['contour']], -1, (255, 0, 0), 3)

            center = best_rgb['center']
            angle = best_rgb['angle']
            cv2.circle(color_img_display, tuple(center), 3, (0, 0, 255), 2)
            r = 30
            cv2.line(color_img_display,
                     tuple(np.int32(center) + np.int32([r * cos(angle), r * sin(angle)])),
                     tuple(np.int32(center) - np.int32([r * cos(angle), r * sin(angle)])),
                     (0, 0, 255),
                     2)
            cv2.putText(color_img_display, "selected", (center[0], center[1]), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0),
                        thickness=2)

            cv2.imshow("hole detection contours_rgb", color_img_display)
            #cv2.moveWindow('camera contours_rgb', size_x, size_y)

            return best_rgb['features']


        # show camera image with annotations

        if FULL_VISUALIZE and not NO_VISUALIZATION:
            #RGB
            hole_image = color_img
            kernel = np.ones((9, 9), 'uint8')
            global_mask_rgb = cv2.erode(global_mask_rgb, kernel)
            hole_image[global_mask_rgb == 0] = 255
            cv2.imshow("holes_rgb", hole_image)
            #cv2.moveWindow('holes_rgb', 0, size_y)

        if not NO_VISUALIZATION:
            cv2.imshow("hole detection contours_rgb", color_img_display)
        #cv2.moveWindow('camera contours_rgb', size_x, size_y)
        self.pose_pub.publish(msg_array)

        # publish to ROS
        small_img = color_img_display  ##detection from depth
        cv_mat = cv.fromarray(small_img)

        img_msg = self.bridge.cv_to_imgmsg(cv_mat, encoding="bgr8")

        self.img_pub.publish(img_msg)


if __name__ == "__main__":
    rospy.init_node("detect_holes_rgb")
    detect = DetectHolesRGB()

    rospy.spin()

    # to be save send 0-twist
    # vs.send_twist(0, 0)
    cv.DestroyAllWindows()

