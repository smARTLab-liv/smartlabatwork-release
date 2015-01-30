#!/usr/bin/env python
import math
import rospy
from sensor_msgs.msg import Image
# import message_filters
from cv_bridge import CvBridge, CvBridgeError

# import thread
import cv
import cv2
import numpy as np
import tf

from sensor_msgs.msg import LaserScan

from math import sin, cos

from slaw_msgs.msg import PoseStampedLabeled, PoseStampedLabeledArray

# from weka_classifier import WekaClassifier

from j48 import j48_depth, j48_july_depth, j48_depth_horizontal
from vision_helpers import add_laser_feature_v2
import vision_constants

from vision_helpers import filter_contours, get_objects, process_depth, pixel_to_m, \
    merge_contours, get_closest, LASER_POS_X, LASER_POS_Y, \
    single_full2crop_coordinates, find_m20_head, rotate_point, m_to_pixel  # size_y, m_to_pixel
from laser_helpers import create_laser_img, \
    set_offsets_to  # OFFSET_X, OFFSET_Y, create_laser_img , set_offset_x, set_offset_
from slaw_srvs.srv import SetSide, SetSideRequest

FULL_VISUALIZE = False
USE_LASER = False
NO_VISUALIZATION = True
# classifier_d = WekaClassifier('/home/youbot/test/depth.arff')
# classifier_r = WekaClassifier('/home/youbot/test/rgb.arff')


class DetectObjectsDepth():
    def __init__(self, training=False):
        # platform orientation is 'front' if platform is in front else 'side'
        res = SetSideRequest()
        res.side = rospy.get_param('platform_direction', 'left')
        set_offsets_to(res)
        self.bridge = CvBridge()
        self.laser_points = None
        self.cur_rgb = None
        self.cur_depth = None
        self.non_crop = None
        self.cur_laser = None
        self.training = training
        if not training:
            # init camera
            rospy.Subscriber("/camera/depth_registered/hw_registered/image_rect_raw", Image, self.cb_depth)
            rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.cb_rgb)
            rospy.Subscriber("/objects_scan", LaserScan, self.cb_laser)
            rospy.Service("/vision/depth/switch_side", SetSide, set_offsets_to)

            # depth_sub = message_filters.Subscriber("/camera/depth_registered/hw_registered/image_rect_raw",Image)
            # rgb_sub = message_filters.Subscriber("/camera/rgb/image_rect_color", Image)
            # self.ts = message_filters.TimeSynchronizer([depth_sub, rgb_sub], 10)
            # self.ts.registerCallback(self.cb_images)

            self.img_pub = rospy.Publisher("/vision/image_depth", Image)
            self.pose_pub = rospy.Publisher("/vision/detected_objects", PoseStampedLabeledArray)
            rate = rospy.Rate(30)

            while not rospy.is_shutdown():
                if self.cur_rgb is None or self.cur_depth is None or (USE_LASER and self.cur_laser is None):
                    rate.sleep()
                    continue
                #if OPENNI_LEGACY:
                depth = self.bridge.imgmsg_to_cv(self.cur_depth, "32FC1")
                #else:
                #    depth = self.bridge.imgmsg_to_cv(self.cur_depth, "8UC1")

                depth = np.array(depth, dtype=np.float32)
                color = self.bridge.imgmsg_to_cv(self.cur_rgb, "bgr8")
                color = np.array(color, dtype=np.uint8)
                self.non_crop = color.copy()
                self.cur_rgb = None
                self.cur_depth = None

                if USE_LASER:
                    laser_img, points = create_laser_img(self.cur_laser)
                    self.laser_points = points
                    #print points
                    self.cur_laser = None

                self.cb_images(depth, color)

                if FULL_VISUALIZE and USE_LASER and not NO_VISUALIZATION:
                    cv2.circle(self.non_crop, (LASER_POS_X, LASER_POS_Y), 2, (0, 0, 255), 2)
                    cv2.imshow("non crop", self.non_crop)

                rate.sleep()


    def cb_rgb(self, msg):
        self.cur_rgb = msg

    def cb_depth(self, msg):
        self.cur_depth = msg

    def cb_images(self, depth_frame, color_frame):
        try:
            # # Crop image
            depth_frame = depth_frame[vision_constants.crop_y[0]:vision_constants.crop_y[1],
                          vision_constants.crop_x[0]:vision_constants.crop_x[1]]
            color_frame = color_frame[vision_constants.crop_y[0]:vision_constants.crop_y[1],
                          vision_constants.crop_x[0]:vision_constants.crop_x[1]]
            if vision_constants.OPENNI_LEGACY:
                depth_frame /= 1000.
            self.process_images(depth_frame, color_frame)
            if not NO_VISUALIZATION:
                cv.WaitKey(5)

        except CvBridgeError, e:
            print e

    def cb_laser(self, msg):
        self.cur_laser = msg

    def process_images(self, depth_img, color_img):
        # make copys

        depth_frame = depth_img.copy()
        color_img_display = color_img.copy()

        frame_gray = cv2.cvtColor(color_img, cv.CV_RGB2GRAY)

        processed_depth = process_depth(depth_frame)

        if FULL_VISUALIZE and not NO_VISUALIZATION:
            cv2.imshow("depth_img", depth_img)
            cv2.imshow("processed_depth", processed_depth)

        # contours
        contours_depth, hierarchy = cv2.findContours(processed_depth,
                                                     cv2.RETR_EXTERNAL,
                                                     cv2.CHAIN_APPROX_SIMPLE)

        # # Filter contours (min max area and min max axis length)
        contours_depth = merge_contours(contours_depth)

        contours_depth, approx_depth = filter_contours(contours_depth)

        size = depth_img.shape
        size = (size[1] - 1, size[0] - 1)

        cross_w = size[0] / 2 + vision_constants.CROSS_OFF_X
        cross_h = size[1] / 2 + vision_constants.CROSS_OFF_Y

        # cross = np.int32([cross_w, cross_h])
        # crosshair
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
            msg_array.detection_type = 'depth'
        global_mask = None
        if FULL_VISUALIZE:
            global_mask = np.zeros(depth_img.shape, dtype=np.uint8)
        objects_depth = get_objects(approx_depth, depth_img, frame_gray, global_mask=global_mask, use_laser=USE_LASER)
        if USE_LASER:
            # laser_point have full image coordinates
            add_laser_feature_v2(objects_depth, self.laser_points)

        if not self.training:
            # # walk through objects
            for i, o in enumerate(objects_depth):
                object_label = "unknown"
                center = o['center']
                angle = o['angle']
                features = o['features']
                cv2.circle(color_img_display, tuple(center), 3, (0, 0, 255), 2)
                if USE_LASER:
                    t = o['laser_area']

                    if features[-1] and FULL_VISUALIZE:

                        # cv2.line(self.non_crop, (LASER_POS_X, LASER_POS_Y),
                        #        (center[0] + crop_x[0], center[1] + crop_y[0]), (0, 0, 255), 3)
                        #cv2.line(color_img_display, (LASER_POS_X - crop_x[0], LASER_POS_Y - crop_y[0]), tuple(center),
                        #         (0, 0, 255), 3)
                        if t is not None:
                            cv2.line(color_img_display, tuple(single_full2crop_coordinates(t[0])),
                                     tuple(single_full2crop_coordinates(t[1])),
                                     (0, 0, 255), 3)
                            cv2.line(color_img_display, tuple(single_full2crop_coordinates(t[1])),
                                     tuple(single_full2crop_coordinates(t[2])),
                                     (0, 0, 255), 3)
                            cv2.line(color_img_display, tuple(single_full2crop_coordinates(t[0])),
                                     tuple(single_full2crop_coordinates(t[2])),
                                     (0, 0, 255), 3)
                            cv2.line(self.non_crop, tuple(t[0]),
                                     tuple(t[1]),
                                     (0, 0, 255), 3)
                            cv2.line(self.non_crop, tuple(t[1]),
                                     tuple(t[2]),
                                     (0, 0, 255), 3)
                            cv2.line(self.non_crop, tuple(t[0]),
                                     tuple(t[2]),
                                     (0, 0, 255), 3)

                    elif FULL_VISUALIZE:
                        # cv2.line(self.non_crop, (LASER_POS_X, LASER_POS_Y),
                        #         (center[0] + crop_x[0], center[1] + crop_y[0]), (255, 0, 0), 3)
                        #cv2.line(color_img_display, (LASER_POS_X - crop_x[0], LASER_POS_Y - crop_y[0]), tuple(center),
                        #         (255, 0, 0), 3)
                        if t is not None:
                            cv2.line(color_img_display, tuple(single_full2crop_coordinates(t[0])),
                                     tuple(single_full2crop_coordinates(t[1])),
                                     (255, 0, 0), 3)
                            cv2.line(color_img_display, tuple(single_full2crop_coordinates(t[1])),
                                     tuple(single_full2crop_coordinates(t[2])),
                                     (255, 0, 0), 3)
                            cv2.line(color_img_display, tuple(single_full2crop_coordinates(t[0])),
                                     tuple(single_full2crop_coordinates(t[2])),
                                     (255, 0, 0), 3)
                            cv2.line(self.non_crop, tuple(t[0]),
                                     tuple(t[1]),
                                     (255, 0, 0), 3)
                            cv2.line(self.non_crop, tuple(t[1]),
                                     tuple(t[2]),
                                     (255, 0, 0), 3)
                            cv2.line(self.non_crop, tuple(t[0]),
                                     tuple(t[2]),
                                     (255, 0, 0), 3)

                r = 30

                # res = classifier_d.find_label(features)
                #print res
                #object_label = res[1]
                object_label = j48_depth_horizontal(features)
                if USE_LASER and object_label in vision_constants.CHECK_LASER:
                    object_label = j48_july_depth(features)

                #print("object %s count: %d" % (object_label, o['laser_counter']))

                if object_label == 'M20_100_h':
                    string_res, res = find_m20_head(o['contour'], center, -angle)
                    #print res
                    if string_res == 'Down':
                        vec = [[0, -1]]
                        rot_vec = np.array(rotate_point(vec, (0, 0), angle))
                        vec = np.int32(rot_vec * 1 / 2. * features[2])
                        cv2.circle(color_img_display, tuple(center + vec), 3, (0, 0, 255), 2)
                        angle += math.pi
                        #print 'Down', vec, angle
                    if string_res == 'Top':
                        vec = [[0, 1]]
                        rot_vec = np.array(rotate_point(vec, (0, 0), angle))
                        vec = np.int32(rot_vec * 1 / 2. * features[2])
                        cv2.circle(color_img_display, tuple(center + vec), 3, (0, 0, 255), 2)
                        #print 'Top'
                        #print vec, angle
                    #cv2.drawContours(color_img_display, [res], -1, (255,0,0), 3)
                    center += m_to_pixel(0.015) * rot_vec

                cv2.drawContours(color_img_display, [o['contour']], -1, 0, 3)

                cv2.line(color_img_display,
                         tuple(np.int32(center) + np.int32([r * cos(angle), r * sin(angle)])),
                         tuple(np.int32(center) - np.int32([r * cos(angle), r * sin(angle)])),
                         (0, 0, 255),
                         2)

                cv2.putText(color_img_display, object_label,
                            (center[0] - len(object_label) * 7, center[1] + 15 + int(features[2] / 2)),
                            cv2.FONT_HERSHEY_PLAIN,
                            1.5, (0, 255, 0), thickness=2)

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

                msg.label = object_label
                msg.detection_type = 'depth'

                msg_array.poses.append(msg)
        if self.training:
            best_depth = get_closest(objects_depth, size, self.training)
            center = best_depth['center']
            angle = best_depth['angle']
            cv2.drawContours(color_img_display, [best_depth['contour']], -1, 0, 3)

            cv2.circle(color_img_display, tuple(center), 3, (0, 0, 255), 2)
            r = 30
            cv2.line(color_img_display,
                     tuple(np.int32(center) + np.int32([r * cos(angle), r * sin(angle)])),
                     tuple(np.int32(center) - np.int32([r * cos(angle), r * sin(angle)])),
                     (0, 0, 255),
                     2)
            cv2.putText(color_img_display, "selected", (center[0], center[1]), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0),
                        thickness=2)

            cv2.imshow("camera contours_depth", color_img_display)
            # cv2.moveWindow('camera contours_depth', size_x, 0)

            return best_depth['features']

        # show camera image with annotations
        if FULL_VISUALIZE and not NO_VISUALIZATION:
            # DEPTH
            object_image = color_img
            kernel = np.ones((9, 9), 'uint8')
            global_mask = cv2.erode(global_mask, kernel)
            object_image[global_mask == 0] = 255
            cv2.imshow("objects_depth", object_image)
            # cv2.moveWindow('objects_depth', 0, 0)

            if USE_LASER:
                for point in self.laser_points:
                    cv2.circle(color_img_display,
                               (point[0] - vision_constants.crop_x[0], point[1] - vision_constants.crop_y[0]), 3,
                               (255, 255, 0), 2)
        # for o in objects_depth:
        # print o
        #print "-"*50
        if not NO_VISUALIZATION:
            cv2.imshow("camera contours_depth", color_img_display)
        #cv2.moveWindow('camera contours_depth', size_x, 0)

        self.pose_pub.publish(msg_array)
        # publish to ROS
        small_img = color_img_display  ##detection from depth
        cv_mat = cv.fromarray(small_img)

        img_msg = self.bridge.cv_to_imgmsg(cv_mat, encoding="bgr8")

        self.img_pub.publish(img_msg)


if __name__ == "__main__":
    rospy.init_node("detect_objects_depth")
    detect = DetectObjectsDepth()

    rospy.spin()

    # to be save send 0-twist
    # vs.send_twist(0, 0)
    cv.DestroyAllWindows()

