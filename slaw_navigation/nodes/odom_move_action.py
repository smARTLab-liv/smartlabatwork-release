#!/usr/bin/env python
import rospy
import actionlib
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from nav_msgs.msg import Odometry
import math
from slaw_actions.msg import *
from slaw_srvs.srv import *

import sys, select
import tf


# Node that sends twist until odom dist is greater than x
RATE = 20 # hz

class OdomMove:
    dist = None
    first_pos = None
    max_dist = None
    is_done = False
    
    def __init__(self):
        self.tfListen = tf.TransformListener()
        rospy.sleep(1)
        self.pub = rospy.Publisher("/cmd_vel", Twist)
        self.start = False
        #rospy.Subscriber("/odom", Odometry, self.cb_odom)
        self.action_server = actionlib.simple_action_server.SimpleActionServer('odom_move', OdomFineAdjustAction, self.execute_cb, False)
        self.action_server.start()        


    def get_own_pose(self, x = 0, y = 0):
        own_pose = PoseStamped()
        own_pose.header.frame_id = '/base_link'
        own_pose.header.stamp = rospy.Time(0)
        own_pose.pose.position.x = x
        own_pose.pose.position.y = y
        own_pose.pose.orientation.w = 1.0
        try:
            result = self.tfListen.transformPose('/odom', own_pose)
            return result
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
    
    def execute_cb(self, goal):

        rate = rospy.Rate(RATE)
        twist_msg = Twist()
        twist_msg.linear.x = goal.target.x
        twist_msg.linear.y = goal.target.y
        
        self.max_dist = goal.max_dist
        self.is_done = False
        counter = 0
        self.first_pos = self.get_own_pose()
        # feedback = OdomFineAdjustActionFeedback()
        self.start = True
        while not self.is_done:
            own_pose = self.get_own_pose()
            dist = self.dist(self.first_pos.pose.position, own_pose.pose.position)
            print dist
            # feedback.dist = dist
            #self.action_server.publish_feedback(feedback)
            if self.action_server.is_preempt_requested():
                #stop
                twist_msg = Twist()
                self.pub.publish(twist_msg)
                result = OdomFineAdjustResult()
                result.success = False
                result.dist = dist
                self.action_server.set_preempted()
                self.action_server.set_aborted(result)
                self.start = False
                return
            
            if dist >= self.max_dist:
                break
            self.pub.publish(twist_msg)
            
            counter += 1 
            rate.sleep()            

        # done & stop
        twist_msg = Twist()
        self.pub.publish(twist_msg)
        rospy.loginfo("done")
        result = OdomFineAdjustResult()
        result.success = True
        result.dist = dist
        self.action_server.set_succeeded(result)
        self.start = False
        
    def dist(self, a, b):
        return math.sqrt(pow(a.x - b.x, 2) + pow(a.y-b.y,2))
        
if __name__ == '__main__':
    rospy.init_node('odom_monitor', anonymous = False)
    
    mon = OdomMove()

    #client = actionlib.SimpleActionClient('odom_move', OdomFineAdjustAction)
    #client.wait_for_server()
    #goal = OdomFineAdjustGoal()
    #goal.target.x = -0.2
    #goal.target.y = 0.0
    #goal.max_dist = 1.00
    #goal.duration = 15
    #client.send_goal_and_wait(goal)

    rospy.spin()
