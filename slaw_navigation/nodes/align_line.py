#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#import thread
import cv
import cv2
import numpy as np
#import tf
import actionlib
import math
from geometry_msgs.msg import Twist
from slaw_actions.msg import AlignLineDistanceAction, AlignLineDistanceResult, AlignLineDistanceGoal
from slaw_srvs.srv import GetDist, GetDistResponse, GetAngle, GetAngleResponse


PIXEL_TO_CM = 8.

MIN = 0.6
MAX = 0.68

eps = 0.5

##crop filter
crop_y = [100, 300]
crop_x = [120, 520]

##For placeing the windows
size_x = crop_x[1] - crop_x[0] + 50
size_y = crop_y[1] - crop_y[0] + 50

FULL_VISUALIZE = False
MAX_TURN = 0.1
MIN_TURN = 0.05

MAX_LIN = 0.1
MIN_LIN = 0.02

EPS_LIN = 0.005

EPS_TH = 0.01

OFFSET_TH = 0./180. * math.pi

OFFSET_LIN = -0.04
MAX_COUNTER = 10
MAX_COUNTER_FAILES = 20

origin = (int(crop_x[1] - crop_x[0])/2+50, int(crop_y[1] - crop_y[0])/2)

point1 = (int(origin[0] + 100*np.sin(math.pi+OFFSET_TH)), int(origin[1] - 100*np.cos(math.pi+OFFSET_TH)))
point2 = (int(origin[0] - 100*np.sin(math.pi+OFFSET_TH)), int(origin[1] + 100*np.cos(math.pi+OFFSET_TH)))


def pixel_to_m(pix):
    return (pix/PIXEL_TO_CM)/100

def m_to_pixel(m):
    return int(m* 100* PIXEL_TO_CM)

class AlignLine():
    def __init__(self):
        self.bridge = CvBridge()
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher("/cmd_twist_move_base", Twist)
        rospy.on_shutdown(self.send_zero_twist)
        self.depth_frame = None
        self.avg_th = 0.0
        self.avg_dist = 0.0
        rospy.Subscriber("/camera/depth_registered/hw_registered/image_rect_raw", Image, self.cb_depth)
        self.action_server = actionlib.simple_action_server.SimpleActionServer('align_line_distance', AlignLineDistanceAction, self.execute_cb, False)

        while self.depth_frame is None:
            if rospy.is_shutdown():
                return
            rospy.logwarn('align line waiting for first depth frame')
            rospy.sleep(0.1)
            
        self.action_server.start()        
        self.dist_server = rospy.Service('/vision/get_dist', GetDist, self.get_dist_to_platform)
        self.angle_server = rospy.Service('/vision/get_angle', GetAngle, self.get_angle_to_platform)



    def send_zero_twist(self):
        self.send_twist(0, 0)
        
    def cb_depth(self, msg):
        self.depth_frame = msg


    def get_angle_to_platform(self, req):
        rate = rospy.Rate(30)

        avg_turn = 0
        counter_failed = 0
        res = GetAngleResponse()
        res.ang = -1
        NUM_READINGS = 10
        for i in xrange(NUM_READINGS):
            while self.depth_frame is None:
                if rospy.is_shutdown():
                    if FULL_VISUALIZE:
                        cv.DestroyAllWindows()
                    return res
                rate.sleep()

            if rospy.is_shutdown() or counter_failed > MAX_COUNTER_FAILES:
                if FULL_VISUALIZE:
                    cv.DestroyAllWindows()
                return res

            depth_frame = self.bridge.imgmsg_to_cv(self.depth_frame, "32FC1")
            self.depth_frame = None
            depth_frame = np.array(depth_frame, dtype=np.float32)
            depth_frame = depth_frame[crop_y[0]:crop_y[1], crop_x[0]:crop_x[1]]
            try:
                suc = self.process_image(depth_frame)
                if not suc:
                    counter_failed += 1.
                    rate.sleep()
                    continue
                cv.WaitKey(5)

            except CvBridgeError, e:
                print e
            avg_turn += math.pi/2. + OFFSET_TH - self.avg_th

            rate.sleep()
        if not counter_failed == NUM_READINGS:
            res.ang = - avg_turn / (NUM_READINGS - counter_failed)
        else:
            res.ang = 0.0
        if FULL_VISUALIZE:
            cv.DestroyAllWindows()

        return res



    def get_dist_to_platform(self, req):
        #side = req.side
        rate = rospy.Rate(20)

        avg_move = 0
        counter_failed = 0
        res = GetDistResponse()
        res.dist = -1
        NUM_READINGS = 10
        for i in xrange(NUM_READINGS):
            while self.depth_frame is None:
                if rospy.is_shutdown():
                    if FULL_VISUALIZE:
                        cv.DestroyAllWindows()
                    return res
                rate.sleep()
                
            if rospy.is_shutdown() or counter_failed > MAX_COUNTER_FAILES:
                if FULL_VISUALIZE:
                    cv.DestroyAllWindows()
                return res
 
            depth_frame = self.bridge.imgmsg_to_cv(self.depth_frame, "32FC1")
            self.depth_frame = None
            depth_frame = np.array(depth_frame, dtype=np.float32)
            depth_frame = depth_frame[crop_y[0]:crop_y[1], crop_x[0]:crop_x[1]]
            try:
                suc = self.process_image(depth_frame)
                if not suc:
                    counter_failed += 1.
                    rate.sleep()
                    continue
                cv.WaitKey(5)
        
            except CvBridgeError, e:
                print e

            dist = pixel_to_m(self.avg_dist)
            avg_move += -OFFSET_LIN - dist
            rate.sleep()
        if not counter_failed == NUM_READINGS:
            res.dist = - avg_move / (NUM_READINGS - counter_failed)
        else:
            res.dist = 0.0
        if FULL_VISUALIZE:
            cv.DestroyAllWindows()

        return res
        
    def execute_cb(self, action):
        side = action.side
        target_dist = action.dist
        
        rate = rospy.Rate(20)
        result = AlignLineDistanceResult()
        done = False
        counter = 0
        counter_failed = 0

        max_counter = MAX_COUNTER
        if action.counts > 0:
            max_counter = action.counts

        eps_lin = EPS_LIN
        eps_th = EPS_TH

        if action.eps_lin > 0:
            eps_lin = action.eps_lin

        if action.eps_theta > 0:
            eps_th = action.eps_theta

        while not done:
            if self.action_server.is_preempt_requested() or rospy.is_shutdown() or counter_failed > max_counter:
                #stop
                self.send_zero_twist()
                result.success = False
                self.action_server.set_aborted(result)
                if FULL_VISUALIZE:
                    cv.DestroyAllWindows()
                return

            if self.depth_frame is None:
                rate.sleep()
                continue
                
            depth_frame = self.bridge.imgmsg_to_cv(self.depth_frame, "32FC1")
            self.depth_frame = None
            depth_frame = np.array(depth_frame, dtype=np.float32)
            depth_frame = depth_frame[crop_y[0]:crop_y[1], crop_x[0]:crop_x[1]]
            try:
                suc = self.process_image(depth_frame)
                if not suc:
                    counter_failed += 1
                    rate.sleep()
                    continue
                else:
                    counter_failed = 0
                cv.WaitKey(5)
        
            except CvBridgeError, e:
                print e

            turn = math.pi/2. + OFFSET_TH  - self.avg_th
            dist = pixel_to_m(self.avg_dist)

            move = -target_dist + OFFSET_LIN + dist
            if FULL_VISUALIZE:
                print move, turn
            if abs(turn) < eps_th:
                turn = 0.0
            elif turn > 0:
                turn = max(MIN_TURN, min(turn, MAX_TURN))
            else:
                turn = min(-MIN_TURN, max(turn, -MAX_TURN))

            if abs(move) < eps_lin:
                move = 0.0
            elif move > 0:
                move = max(MIN_LIN, min(move, MAX_LIN))
            else:
                move = min(-MIN_LIN, max(move, -MAX_LIN))
                
            self.send_twist(move, turn, side = side)
            if move == 0.0 and turn == 0.0:
                counter += 1
                if counter > MAX_COUNTER:
                    done = True
            else:
                counter = 0
            rate.sleep()
            
        self.send_zero_twist()
        result.success = True
        self.action_server.set_succeeded(result)
        if FULL_VISUALIZE:
            cv.DestroyAllWindows()


    def intersection(self, o1, p1, o2, p2):
        o1 = np.array(o1)
        p1 = np.array(p1)
        o2 = np.array(o2)
        p2 = np.array(p2)

        x = o2 - o1
        d1 = p1 - o1
        d2 = p2 - o2
        cross = d1[0] * d2[1] - d1[1] * d2[0]
        if (abs(cross) < 0.0001):
            return None

        t1 = (x[0] * d2[1] - x[1] * d2[0]) / cross
        return o1 + d1 * t1
        
        
    def filter_low_high(self, frame, min_val, max_val):
        low_ids = frame < min_val
        high_ids = frame > max_val

        frame[low_ids] = min_val
        frame[high_ids] = min_val
        return frame

    def process_depth(self, depth_img):
        ## Filter NaN
        idx = np.isnan(depth_img)
        depth_img[idx] = 0
        ## Convert to UINT8 image

        depth_img = self.filter_low_high(depth_img, MIN, MAX)
        depth_img = depth_img/(MAX) * 255
        depth_img = np.uint8(depth_img)
        depth_img = cv2.medianBlur(depth_img, 9)

        # frame_filter = cv2.Canny(depth_img,0,50)

        #cv2.imshow('canny', frame_filter)
        #cv.WaitKey(0)
        frame_filter = cv2.adaptiveThreshold(depth_img,
                                            255,
                                            #cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                             cv2.ADAPTIVE_THRESH_MEAN_C,
                                             cv2.THRESH_BINARY,

                                            11,  # neighbourhood
                                            2)

        ## Invert Colors
        cv2.bitwise_not(frame_filter, frame_filter)
        kernel = np.ones((3, 3), 'uint8')
        frame_filter = cv2.erode(frame_filter, kernel)

        return frame_filter

    def wrap_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

    def process_image(self, depth_img):
        depth_frame = depth_img.copy()
        processed_depth = self.process_depth(depth_frame)

        if FULL_VISUALIZE:
            cv2.imshow("camera_gray", depth_img)
            cv2.imshow("processed_depth", processed_depth)
        #processed_depth = cv2.Canny(processed_depth,0,150,apertureSize = 3)
        #cv2.imshow("camera_canny", edges)

        lines = cv2.HoughLines(processed_depth,1,np.pi/360,150)  #80 foor canny

        cv2.line(depth_frame,point1,point2,(0,0,255),2)
                
        avg_th = 0
        avg_dist = 0
        count = 0
        if lines is not None:
            for rho,theta in lines[0]:
                #print theta
                if theta > math.pi/2 + eps or theta < math.pi/2 - eps:
                    continue
                count += 1
                avg_th += theta
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                if FULL_VISUALIZE:
                    cv2.line(depth_frame,(x1,y1),(x2,y2),(0,0,255),2)
                dist = self.intersection(point1, point2, (x0, y0), (x1, y1))
                avg_dist += dist[1]

            if count == 0:
                return False
            avg_th /= count
            avg_dist /= count
        else:
            #cv.WaitKey(0)
            return False
        self.avg_dist = avg_dist
        self.avg_th = avg_th
        #print avg_dist, avg_th
        if FULL_VISUALIZE:
            cv2.imshow("camera_gray2", depth_frame)
        return True
            
    def send_twist(self, lin, ang, side = 'left'):
        msg = Twist()
        if side == 'left':
            msg.linear.y = lin
        else:
            msg.linear.x = lin
        msg.angular.z = ang
        self.cmd_vel_pub.publish(msg)
            
if __name__ == "__main__":
    rospy.init_node("align_line")
    align = AlignLine()
    # client = actionlib.SimpleActionClient('align_line_distance', AlignLineDistanceAction)
    # client.wait_for_server()
    # goal = AlignLineDistanceGoal()
    # goal.dist = 0.02
    # goal.side = 'left'
    # client.send_goal_and_wait(goal)
    # print client.get_result()

    #to be save send 0-twist
    #align.send_twist(0, 0)
    rospy.spin()
    if FULL_VISUALIZE:
        cv.DestroyAllWindows()

