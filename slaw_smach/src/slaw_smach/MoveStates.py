#!/usr/bin/env python
import math
from math import pi
import numpy as np

import rospy
import smach
import actionlib
from std_srvs.srv import EmptyResponse, Empty
import tf
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from slaw_srvs.srv import *
from slaw_actions.msg import *
from geometry_msgs.msg import PoseStamped, Twist, Pose
from smach_helpers import get_pickup_side, init_helpers, backplate_object_msg_to_dict


NORTH = 0.0

DISTANCE_TO_PLATFORM = 0.05

MIN_DIST = 0.01
MAX_DIST = 0.2

MAX_TRIES = None
TIME_OUT = None
NO_MOVE_BASE = None
NO_FINE_ADJUST = None

place_offsets_position = [-0.12, 0, 0.12]


def init_globals():
    global MAX_TRIES, TIME_OUT, NO_MOVE_BASE, NO_FINE_ADJUST
    if MAX_TRIES is None:
        init_helpers()
        MAX_TRIES = rospy.get_param("smach/max_tries", 5)
        TIME_OUT = rospy.get_param("smach/time_out", 5.0)
        NO_MOVE_BASE = rospy.get_param("smach/no_move_base", False)
        NO_FINE_ADJUST = rospy.get_param("smach/no_fine_adjust", False)


def create_goal_message(goal):
    goal_msg = MoveBaseGoal()


    # goal_msg.target_pose.pose.position.x = goal["x"]
    # goal_msg.target_pose.pose.position.y = goal["y"]
    # if 'theta' in goal.keys():
    # q = tf.transformations.quaternion_from_euler(0,0, goal["theta"], axes='sxyz')
    # goal_msg.target_pose.pose.orientation.x = q[0]
    # goal_msg.target_pose.pose.orientation.y = q[1]
    # goal_msg.target_pose.pose.orientation.z = q[2]
    #    goal_msg.target_pose.pose.orientation.w = q[3]
    #else:
    #   rospy.logwarn("no theta set, defaulting to 0")
    #   goal_msg.target_pose.pose.orientation.w = 1.0
    goal_msg.target_pose.pose = goal

    goal_msg.target_pose.header.frame_id = "/map"
    goal_msg.target_pose.header.stamp = rospy.Time.now()
    return goal_msg


def dir_to_quaternion(direction):
    theta = dir_to_theta(direction)
    q = tf.transformations.quaternion_from_euler(0, 0, theta, axes='sxyz')
    return q


def dir_to_theta(direction):
    theta = None
    if direction == 'N':
        theta = .0 + NORTH
    elif direction == 'W':
        theta = .5 * pi + NORTH
    elif direction == 'S':
        theta = pi + NORTH
    elif direction == 'E':
        theta = 1.5 * pi + NORTH
    else:
        rospy.logerr('Direction "%s" is unknown' % (direction))
    return theta


class AlignToObjectState(smach.State):
    def __init__(self, eps_lin=0.01, eps_theta=0.02, counter=5):
        smach.State.__init__(self, outcomes=['reached', 'failed'], input_keys=['point_in', 'pose_in'])
        init_globals()

        self.dist_platform = rospy.ServiceProxy("/vision/get_dist", GetDist)
        self.align_client = actionlib.SimpleActionClient('align_line_distance', AlignLineDistanceAction)
        self.odom_fine_client = actionlib.SimpleActionClient('odom_move', OdomFineAdjustAction)
        self.eps_lin = eps_lin
        self.eps_theta = eps_theta
        self.counter = counter
        if not NO_FINE_ADJUST:
            self.odom_fine_client.wait_for_server()
            self.align_client.wait_for_server()

    def execute(self, userdata):
        pose = userdata.pose_in
        offset = userdata.point_in

        side = get_pickup_side(pose['name'])
        cur_dist = None
        try:
            cur_dist = self.dist_platform('side')
            # print poseLoc
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return 'failed'

        goal = AlignLineDistanceGoal()
        goal.side = side

        dist = cur_dist.dist - offset.position.x

        goal.dist = max(MIN_DIST, min(dist, MAX_DIST))
        goal.eps_lin = self.eps_lin
        goal.eps_theta = self.eps_theta
        goal.counts = self.counter
        goal_x = OdomFineAdjustGoal()
        if side == "left":
            goal_x.target.x = -np.sign(offset.position.y) * 0.1
        else:
            goal_x.target.y = np.sign(offset.position.y) * 0.1

        goal_x.max_dist = abs(offset.position.y)
        goal_x.duration = 5

        if NO_FINE_ADJUST:
            rospy.sleep(2)
            # return 'failed'
            return 'reached'

        self.odom_fine_client.send_goal_and_wait(goal_x)

        self.align_client.send_goal_and_wait(goal)
        if self.align_client.get_state() == GoalStatus.SUCCEEDED:
            return 'reached'
        else:
            return 'failed'


class LeaveLocationState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.leave_destination = rospy.ServiceProxy('/smach/destination_manager_leave_location', Empty)

    def execute(self, userdata):

        try:
            self.leave_destination()
        except Exception as e:
            print e
        return 'done'


class DestinationPlaceManager(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['nothing_left', 'place'], input_keys=['pose_in'],
                             output_keys=['object_out', 'point_out', 'backplate_pose_out'])
        init_globals()
        self.get_objects_on_plate = rospy.ServiceProxy("/backplate_manager/get_objects_from_plate",
                                                       BackplateStateStatus)
        self.odom_fine_client = actionlib.SimpleActionClient('odom_move', OdomFineAdjustAction)
        self.visitedPlaces = {}
        self.move_x = [0.10, 0.40]
        self.speed = 0.15
        self.last_location = None
        rospy.Service('/smach/reset_destination_manager', Empty, self.reset)
        rospy.Service('/smach/destination_manager_leave_location', Empty, self.leave_location)
        print 'destination manager started'

    def leave_location(self, req):
        if self.last_location is not None:
            if self.last_location in self.visitedPlaces:
                print str(self.last_location) + ': alreadyDriven reset'
                self.visitedPlaces[self.last_location]['alreadyDriven'] = 0
        return EmptyResponse()

    def reset(self, req):
        self.visitedPlaces.clear()
        return EmptyResponse()

    def execute(self, userdata):
        place_label = userdata.pose_in['name']
        self.last_location = place_label
        self.init_driven_counter_and_placed_items(place_label)
        res = self.get_objects_on_plate()

        has_object_to_place = False
        for obj in res.objects_on_plate:
            if obj.destination_loc == place_label:
                has_object_to_place = True
                obj_dict = backplate_object_msg_to_dict(obj)
                userdata.object_out = obj_dict
                userdata.backplate_pose_out = obj_dict['backplate_location']
                pose = Pose()
                side = get_pickup_side(place_label)
                if side == 'left':
                    pose.position.y = place_offsets_position[self.visitedPlaces[place_label]['placedItems'] % 3]
                else:
                    pose.position.y = place_offsets_position[2 - self.visitedPlaces[place_label]['placedItems'] % 3]

                pose.orientation.w = 1
                userdata.point_out = pose

        if not has_object_to_place:
            self.visitedPlaces[place_label]['alreadyDriven'] = 0
            # self.visitedPlaces[place_label]['placedItems'] += 1
            return 'nothing_left'

        self.driveToPos(place_label)
        self.visitedPlaces[place_label]['placedItems'] += 1
        return 'place'


    def init_driven_counter_and_placed_items(self, place_label):

        if place_label not in self.visitedPlaces.keys():
            self.visitedPlaces[place_label] = {}
            self.visitedPlaces[place_label]['alreadyDriven'] = 0
            self.visitedPlaces[place_label]['placedItems'] = 0


    def driveToPos(self, place_label):
        index_to_drive = min((int(self.visitedPlaces[place_label]['placedItems'] / 3)) + 1, len(self.move_x))
        already_driven = self.visitedPlaces[place_label]['alreadyDriven']

        side = get_pickup_side(place_label)

        goal_x = OdomFineAdjustGoal()
        if side == "left":
            goal_x.target.x = -self.speed
        else:
            goal_x.target.y = -self.speed

        goal_x.max_dist = sum(self.move_x[already_driven: index_to_drive])
        if goal_x.max_dist > 0:
            self.visitedPlaces[place_label]['alreadyDriven'] = index_to_drive

        self.odom_fine_client.send_goal(goal_x)


class AlignToPlatformState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached', 'failed'], input_keys=['pose_in'])
        init_globals()
        self.align_client = actionlib.SimpleActionClient('align_line_distance', AlignLineDistanceAction)

        if not NO_FINE_ADJUST:
            self.align_client.wait_for_server()

    def execute(self, userdata):
        pose = userdata.pose_in
        side = get_pickup_side(pose['name'])

        goal = AlignLineDistanceGoal()
        goal.side = side
        goal.dist = DISTANCE_TO_PLATFORM
        print goal
        if NO_FINE_ADJUST:
            rospy.sleep(2)
            # return 'failed'
            return 'reached'

        self.align_client.send_goal_and_wait(goal)
        if self.align_client.get_state() == GoalStatus.SUCCEEDED:
            return 'reached'
        else:
            return 'failed'


class RecoverState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.switch_off = rospy.ServiceProxy('/scan_registration/switchOffForcefield', switchOff)
        self.pub = rospy.Publisher('/cmd_vel', Twist)

    def sendPause(self, pause):
        rospy.wait_for_service('/scan_registration/switchOffForcefield')
        try:
            self.switch_off(pause)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def execute(self, userdata):
        # pose = userdata.pose_in
        # userdata.pose_out = pose
        self.sendPause(False)
        rospy.sleep(2.0)
        self.sendPause(True)
        self.pub.publish(Twist())
        return 'done'


class MoveStateUserData(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached', 'not_reached', 'failed'], input_keys=['pose_in'])

        init_globals()
        self.tfListen = tf.TransformListener()
        rospy.sleep(1)

        # #start movebasec client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        if not NO_MOVE_BASE:
            self.client.wait_for_server()

        self.locService = rospy.ServiceProxy("location_service/get_location", GetLocation)

        self.counter = MAX_TRIES

    # self.old_state = None

    def get_own_pose(self, x=0, y=0):
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


    def dist(self, a, b):
        return math.sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2))


    def execute(self, userdata):
        # rospy.loginfo('Executing state "%s"'%self.label)

        pose = userdata.pose_in

        # For BNT:
        if pose['type'] == 'BNT':
            suffix = ''.join(['_', pose['dir']])
        else:
            suffix = ''

        if NO_MOVE_BASE:
            rospy.sleep(2)
            self.counter = MAX_TRIES
            return 'reached'

        try:
            rospy.wait_for_service('location_service/get_location', timeout=TIME_OUT)
            goal = self.locService(''.join([pose['name'], suffix]))  # string name
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return 'failed'

        print goal.locations[0].name
        goal_msg = create_goal_message(goal.locations[0].pose)

        null_speed_counter = 0
        max_null_speed_counter = 20

        self.client.send_goal(goal_msg)

        # self.client.wait_for_result(rospy.Duration(TIME_OUT))
        r = rospy.Rate(5)
        self.first_pose = self.get_own_pose()
        r.sleep()
        while not self.client.get_state() == GoalStatus.SUCCEEDED:
            # move
            own_pose = self.get_own_pose()
            dist = self.dist(self.first_pose.pose.position, own_pose.pose.position)
            # print dist
            ### TODO maybe add yaw
            if dist > 0.0001:
                #print 'moved'
                self.first_pose = own_pose
            else:
                if null_speed_counter > max_null_speed_counter:  #remove False
                    if self.counter > 0:
                        self.counter -= 1
                        self.client.cancel_all_goals()
                        return 'not_reached'
                    else:
                        self.counter = MAX_TRIES
                        return 'failed'
                else:
                    print "Not Moved"
                    null_speed_counter += 1

            r.sleep()


        # rospy.logerr(self.client.get_state())
        if self.client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("reached goals")
            self.client.cancel_all_goals()
            self.counter = MAX_TRIES
            return 'reached'

        else:
            if self.counter > 0:
                self.counter -= 1
                self.client.cancel_all_goals()
                return 'not_reached'
            else:
                self.counter = MAX_TRIES
                return 'failed'


class MoveBackWithAlign(smach.State):
    def __init__(self, distance, speed):
        smach.State.__init__(self, outcomes=['done'], input_keys=['pose_in'])
        init_globals()
        self.distance = distance
        self.speed = speed
        self.odom_fine_client = actionlib.SimpleActionClient('odom_move', OdomFineAdjustAction)
        self.get_angle = rospy.ServiceProxy("/vision/get_angle", GetAngle)
        self.align_client = actionlib.SimpleActionClient('align_line_distance', AlignLineDistanceAction)
        self.tfListen = tf.TransformListener()
        rospy.sleep(1)

        if not NO_FINE_ADJUST:
            self.odom_fine_client.wait_for_server()

    def dist(self, a, b):
        return math.sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2))


    def get_own_pose(self, x=0, y=0):
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

    def execute(self, userdata):
        pose = userdata.pose_in

        side = get_pickup_side(pose['name'])

        goal = OdomFineAdjustGoal()
        if side == "left":
            goal.target.x = - self.speed
        else:
            goal.target.y = - 1.5 * self.speed

        goal.max_dist = self.distance
        goal.duration = 5

        print goal
        if NO_FINE_ADJUST:
            rospy.sleep(2)
            # return 'failed'
            return 'done'

        # self.odom_fine_client.send_goal_and_wait(goal)
        self.odom_fine_client.send_goal(goal)
        r = rospy.Rate(10)
        traveled_dist = 0
        first_pose = self.get_own_pose()
        while not self.odom_fine_client.get_state() == GoalStatus.SUCCEEDED:
            res = self.get_angle(side)
            if abs(res.ang) > math.pi / 180. * 5:
                self.odom_fine_client.cancel_all_goals()
                # result = self.odom_fine_client.()
                # print result
                own_pose = self.get_own_pose()
                dist = self.dist(first_pose.pose.position, own_pose.pose.position)

                traveled_dist = dist
                goal_align = AlignLineDistanceGoal()
                goal_align.side = side
                goal_align.dist = DISTANCE_TO_PLATFORM
                # goal.eps_lin = self.eps_lin
                # goal.eps_theta = self.eps_theta
                # goal.counts = self.counter
                self.align_client.send_goal_and_wait(goal_align)
                #set new goal distance
                goal.max_dist = self.distance - traveled_dist
                self.odom_fine_client.send_goal(goal)
            r.sleep()

        return 'done'


class MoveBack(smach.State):
    def __init__(self, distance, speed):
        smach.State.__init__(self, outcomes=['done'], input_keys=['pose_in'])
        init_globals()
        self.distance = distance
        self.speed = speed
        self.odom_fine_client = actionlib.SimpleActionClient('odom_move', OdomFineAdjustAction)

        if not NO_FINE_ADJUST:
            self.odom_fine_client.wait_for_server()

    def execute(self, userdata):
        pose = userdata.pose_in

        side = get_pickup_side(pose['name'])

        goal = OdomFineAdjustGoal()
        if side == "left":
            goal.target.x = - self.speed
        else:
            goal.target.y = - self.speed

        goal.max_dist = self.distance
        goal.duration = 5

        print goal
        if NO_FINE_ADJUST:
            rospy.sleep(2)
            # return 'failed'
            return 'done'

        self.odom_fine_client.send_goal_and_wait(goal)
        return 'done'


class MoveTowardsPlatform(smach.State):
    def __init__(self, distance, speed):
        smach.State.__init__(self, outcomes=['done'], input_keys=['pose_in'])
        init_globals()
        self.distance = distance
        self.speed = speed
        self.odom_fine_client = actionlib.SimpleActionClient('odom_move', OdomFineAdjustAction)

        if not NO_FINE_ADJUST:
            self.odom_fine_client.wait_for_server()

    def execute(self, userdata):
        pose = userdata.pose_in

        side = get_pickup_side(pose['name'])

        goal = OdomFineAdjustGoal()
        if side == "left":
            goal.target.y = self.speed
        else:
            goal.target.x = self.speed

        goal.max_dist = self.distance
        goal.duration = 5

        print goal
        if NO_FINE_ADJUST:
            rospy.sleep(2)
            # return 'failed'
            return 'done'

        self.odom_fine_client.send_goal_and_wait(goal)
        return 'done'


class ScanMatcher(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached', 'not_reached', 'failed'], input_keys=['pose_in'])

        init_globals()
        self.client = actionlib.SimpleActionClient('registration_fine_adjust', RegistrationFineAdjustAction)

        self.locService = rospy.ServiceProxy("location_service/get_location", GetLocation)
        if not NO_FINE_ADJUST:
            self.client.wait_for_server()
        self.counter = MAX_TRIES


    def execute(self, userdata):

        pose = userdata.pose_in
        suffix = 'Pickup'  # for CBT, Pickup or Place always scanmatch to begin of platform

        # For BNT:
        if pose['type'] == 'BNT':
            suffix = pose['dir']

        if NO_FINE_ADJUST:
            rospy.sleep(2)
            # if self.counter > 0:
            # self.counter -= 1
            # self.client.cancel_all_goals()
            # return 'not_reached'
            #else:
            #    self.counter = MAX_TRIES
            #    return 'failed'

            #return 'failed'
            return 'reached'

        try:
            poseLoc = self.locService(''.join([pose['name'], '_', suffix]))  # string name
            # print poseLoc
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return 'failed'

        if len(poseLoc.locations) > 0:
            goal = RegistrationFineAdjustGoal()
            goal.front = poseLoc.locations[0].scan_front
            goal.rear = poseLoc.locations[0].scan_rear
            goal.x_thresh = 0.01
            goal.y_thresh = 0.01
            goal.theta_thresh = 0.01
            goal.duration = TIME_OUT
        else:
            print "No Location recorded at that pose"
            return 'failed'

        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(TIME_OUT))

        result = self.client.get_result()
        rospy.loginfo("%s" % str(result))
        if self.client.get_state() == GoalStatus.SUCCEEDED:
            self.client.cancel_all_goals()
            self.counter = MAX_TRIES
            return 'reached'

        else:
            # TODO check if stil "Close"
            if self.counter > 0:
                self.counter -= 1
                self.client.cancel_all_goals()
                return 'not_reached'
            else:
                self.counter = MAX_TRIES
                return 'failed'
