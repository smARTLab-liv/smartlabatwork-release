#!/usr/bin/env python
import rospy
import tf
import math
import actionlib
import smach
import copy
import numpy as np
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty
from slaw_actions.msg import *
from slaw_srvs.srv import *
from slaw_msgs.msg import *
from geometry_msgs.msg import Pose
from arm_helpers import grip_point, init_arm_helpers, call_ik_solver, get_place_height_for_obj
from smach_helpers import get_objects_on_plate, backplate_object_msg_to_dict, init_helpers, get_pickup_side, \
    publish_status

TIME_DIFF = 0.2
base_frame = '/arm_base_link'
NO_MOVE_BASE = None
NO_FINE_ADJUST = None
NO_VISION = None
loc_service = None
MAX_DETECT = 30


def init_globals_detect():
    global NO_MOVE_BASE, NO_FINE_ADJUST, NO_VISION, loc_service
    if NO_MOVE_BASE is None:
        init_arm_helpers()
        init_helpers()
        NO_MOVE_BASE = rospy.get_param("smach/no_move_base", False)
        NO_FINE_ADJUST = rospy.get_param("smach/no_fine_adjust", False)
        NO_VISION = rospy.get_param("smach/no_vision", False)


class VerifyHoleState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed', 'too_far'], input_keys=['pose_in'],
                             output_keys=['point_out', 'object_out', 'backplate_pose_out'])
        init_globals_detect()
        self.switch_on = rospy.ServiceProxy("/vision/hole_manager/switch_on", SwitchOnForLocation)
        self.switch_off = rospy.ServiceProxy("/vision/hole_manager/switch_off", Empty)
        self.verify_object = rospy.ServiceProxy("/vision/hole_manager/verify_hole", GetObjectAtLocation)

    def execute(self, userdata):
        rospy.wait_for_service("/vision/hole_manager/switch_on")
        pose = userdata.pose_in
        try:
            print 'switch on'
            self.switch_on(pose['name'])
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return 'failed'
        rospy.sleep(1.5)
        rospy.wait_for_service("/vision/object_manager/switch_off")
        try:
            print 'switch off'
            self.switch_off()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return 'failed'

        objects_on_plate = get_objects_on_plate()
        holes = []
        holes_to_obj = {}
        for obj in objects_on_plate:
            param_obj = rospy.get_param(obj.label)
            h = param_obj['hole']
            holes_to_obj[h] = obj.label
            holes.append(h)
        hole = None
        req = GetObjectAtLocationRequest()
        req.loc = pose['name']
        req.objects_to_pick = holes
        print req
        try:
            rospy.wait_for_service("/vision/object_manager/get_best")
            hole = self.verify_object(req).object
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return 'failed'

        if hole.label == 'None':
            return 'failed'

        object_name = holes_to_obj[hole.label]
        print object_name
        height = get_place_height_for_obj(object_name)

        p = hole.pose.pose.position
        test_point = [grip_point[0] + p.x, p.y, grip_point[2] + height]
        side = get_pickup_side(pose['name'])

        conf = call_ik_solver(test_point, side=side)
        if len(conf) == 0:
            return 'too_far'

        object_out = None
        for o in objects_on_plate:
            if o.label == object_name:
                object_out = o
                break
        print object_out
        userdata.object_out = backplate_object_msg_to_dict(object_out)
        userdata.backplate_pose_out = object_out.place_loc
        userdata.point_out = hole.pose.pose
        print object_out, hole.label
        return 'success'


class VerifyObjectState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['RV_h', 'RV_v', 'black_gray', 'success', 'cannot_place', 'failed'],
                             input_keys=['pose_in'], output_keys=['point_out', 'object_out', 'backplate_pose_out'])
        init_globals_detect()
        self.switch_on = rospy.ServiceProxy("/vision/object_manager/switch_on", SwitchOnForLocation)
        self.switch_off = rospy.ServiceProxy("/vision/object_manager/switch_off", Empty)
        self.verify_object = rospy.ServiceProxy("/vision/object_manager/verify_object", GetObjectAtLocation)
        self.can_place_object = rospy.ServiceProxy("/backplate_manager/can_place_specific_object", CanPlaceObject)
        self.get_best_backplate_location = rospy.ServiceProxy("/backplate_manager/get_best_place_location",
                                                              GetBestPlaceLocation)

    def execute(self, userdata):
        rospy.wait_for_service("/vision/object_manager/switch_on")
        pose = userdata.pose_in
        try:
            print 'switch on'
            self.switch_on(pose['name'])
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return 'failed'

        rospy.sleep(1.5)
        rospy.wait_for_service("/vision/object_manager/switch_off")
        try:
            print 'switch off'

            self.switch_off()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return 'failed'

        param_pose = rospy.get_param(pose['name'])
        param_objects = param_pose['objects']

        objects = [x['name'] for x in param_objects]
        # print "OBJECTS AT LOCATION :", objects

        obj = None
        req = GetObjectAtLocationRequest()
        req.loc = pose['name']
        req.objects_to_pick = objects
        print req
        try:
            rospy.wait_for_service("/vision/object_manager/get_best")
            obj = self.verify_object(req).object
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return 'failed'

        if obj.label == 'None':
            return 'failed'


        # #Check if can place object
        res = self.can_place_object(obj.label)
        if not res.can_place:
            return 'cannot_place'


        # found = False
        # Check which object it is


        print obj.object
        if obj.candidates == 1:
            if obj.object == 'RV20':
                if 'V20' in objects:
                    obj.object = 'V20'
                    obj.label = obj.label.replace('RV20', 'V20')
                if 'R20' in objects:
                    obj.object = 'R20'
                    obj.label = obj.label.replace('RV20', 'R20')
            if obj.object == 'F20_20':
                if 'F20_20_G' in objects:
                    obj.object = 'F20_20_G'
                    obj.label = obj.label.replace('F20_20', 'F20_20_G')
                if 'F20_20_B' in objects:
                    obj.object = 'F20_20_B'
                    obj.label = obj.label.replace('F20_20', 'F20_20_B')
            if obj.object == 'S40_40':
                if 'S40_40_G' in objects:
                    obj.object = 'S40_40_G'
                    obj.label = obj.label.replace('S40_40', 'S40_40_G')
                if 'S40_40_B' in objects:
                    obj.object = 'S40_40_B'
                    obj.label = obj.label.replace('S40_40', 'S40_40_B')

        elif 'RV20_h' in obj.label:
            userdata.object_out = {'name': obj.object, 'label': obj.label}
            userdata.point_out = obj.pose.pose
            return 'RV_h'
        elif 'RV20_v' in obj.label:
            userdata.object_out = {'name': obj.object, 'label': obj.label}
            userdata.point_out = obj.pose.pose
            return 'RV_v'
        elif 'S40_40_v' in obj.label or 'F20_20_v' in obj.label:
            userdata.object_out = {'name': obj.object, 'label': obj.label}
            userdata.point_out = obj.pose.pose
            return 'black_grey'

        print obj
        res = self.get_best_backplate_location(obj_name=obj.label, objects_to_pick=objects)
        userdata.backplate_pose_out = res.place_loc

        object_out = {}
        for o in param_objects:
            if obj.object == o['name']:
                object_out = o
                break

        object_out['label'] = obj.label
        userdata.object_out = object_out
        userdata.point_out = obj.pose.pose

        print object_out

        return 'success'


class SwitchObjectHoleManagerOnState(smach.State):
    def __init__(self, holes=False):
        smach.State.__init__(self, outcomes=['done'], input_keys=['pose_in'])
        init_globals_detect()
        self.topic_str = ''
        self.holes = holes
        if not holes:
            self.topic_str = '/vision/object_manager/switch_on'
            self.reset = rospy.ServiceProxy('/vision/object_manager/reset', Empty)
        else:
            self.topic_str = '/vision/hole_manager/switch_on'
            self.reset = rospy.ServiceProxy('/vision/hole_manager/reset', Empty)
        self.switch_on = rospy.ServiceProxy(self.topic_str, SwitchOnForLocation)
        self.vision_rgb_switch_side = rospy.ServiceProxy("/vision/rgb/switch_side", SetSide)
        self.vision_depth_switch_side = rospy.ServiceProxy("/vision/depth/switch_side", SetSide)

    def execute(self, userdata):
        pose = userdata.pose_in
        side = get_pickup_side(pose['name'])
        try:
            self.reset()
            rospy.wait_for_service(self.topic_str)
            self.vision_rgb_switch_side(side=side)
            self.vision_depth_switch_side(side=side)

            self.switch_on(userdata.pose_in['name'])
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        rospy.sleep(1.0)
        if self.holes:
            publish_status('I am scanning the service area <br> for holes.')
        else:
            publish_status('I am scanning the service area <br> for objects.')
        return 'done'


class SwitchObjectHoleManagerOffState(smach.State):
    def __init__(self, holes=False):
        smach.State.__init__(self, outcomes=['done'])
        init_globals_detect()
        self.topic_str = ''
        if not holes:
            self.topic_str = '/vision/object_manager/switch_off'
        else:
            self.topic_str = '/vision/hole_manager/switch_off'
        self.switch_off = rospy.ServiceProxy(self.topic_str, Empty)

    def execute(self, userdata):
        try:
            rospy.wait_for_service(self.topic_str)
            self.switch_off()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        return 'done'


class RV20CheckVision(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['correct', 'incorrect'], input_keys=['pose_in', 'object_in'],
                             output_keys=['object_out', 'backplate_pose_out'])
        self.check_rv20 = rospy.ServiceProxy('/vision/check_rv20', CheckRV20)
        self.get_best_backplate_location = rospy.ServiceProxy("/backplate_manager/get_best_place_location",
                                                              GetBestPlaceLocation)


    def execute(self, userdata):
        param_objects = rospy.get_param(userdata.pose_in['name'])['objects']

        objects_to_pick = [x['name'] for x in param_objects]

        check = self.check_rv20()
        publish_status('I am checking if it is <br> a R20 or V20.')

        if check.result in objects_to_pick:
            obj = userdata.object_in
            for o in param_objects:
                if o['name'] == check.result:
                    object_out = o
                    break
            object_out['label'] = obj['label'].replace('RV20', check.result)
            userdata.object_out = object_out
            res = self.get_best_backplate_location(obj_name=object_out['label'], objects_to_pick=objects_to_pick)
            userdata.backplate_pose_out = res.place_loc

            return 'correct'

        userdata.object_out = userdata.object_in
        userdata.backplate_pose_out = []
        return 'incorrect'


class ScanForObjectCBT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        init_globals_detect()

        # rospy.wait_for_service('/cbt_start')
        self.start_service = rospy.ServiceProxy('/cbt_start', Empty)

        rospy.Subscriber('/cbt_status', Bool, self.cb_running)
        self.is_active = False
        self.is_done = False

    def cb_running(self, msg):
        if not self.is_active:
            return

        if not msg.data:  # not running
            self.is_done = True

    def execute(self, userdata):
        self.is_active = True
        self.is_done = False

        # start visual servoing
        self.start_service()

        # wait before we until at least one frame
        rospy.sleep(1)

        r = rospy.Rate(10)
        while not self.is_done:
            r.sleep()

        # wait after gripper closed
        # rospy.sleep(4)

        return 'success'


# ### Unused

class ScanForHoles(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed', 'nothing_found'], input_keys=['pose_in', 'object_in'],
                             output_keys=['object_out', 'pose_out', 'point_out'])
        # self.tfListen = tf.TransformListener()
        # listen for some time
        # self.classes = rospy.get_param("classes")
        # self.objectLocations = {}

        init_globals_detect()
        self.odom_fine_client = actionlib.SimpleActionClient('odom_move', OdomFineAdjustAction)
        if not NO_FINE_ADJUST:
            self.odom_fine_client.wait_for_server()

        self.current_object = None
        self.current_hole = None

        rospy.Subscriber('/vision/detected_object', PoseStampedLabeled, self.cb_detect_object)
        rospy.Subscriber('/vision/detected_hole', PoseStampedLabeled, self.cb_detect)
        self.is_active = False
        #self.copying = False
        #rospy.sleep(1)

    def cb_detect_object(self, msg):
        if self.is_active:
            self.current_object = msg


    def cb_detect(self, msg):
        if self.is_active:
            self.current_hole = msg


    def acceptHole(self, hole):
        now = rospy.Time.now()
        if self.current_hole is None or (now - self.current_hole.pose.header.stamp).to_sec() > 0.5:
            return False
        if hole in ['M20']:
            if self.current_object is not None and (now - self.current_object.pose.header.stamp).to_sec() > 0.2:
                if self.current_object.label in ['M20']:
                    return True

        if not self.current_hole.label == hole:
            return False
        # if hole in ['RV20']:
        # if self.current_hole.pose.pose.position.y < 0.12 and self.current_hole.pose.pose.position.y > -0.06:
        # return True
        # return False

        #else:
        if self.current_hole.pose.pose.position.y < 0.06 and self.current_hole.pose.pose.position.y > -0.06:
            return True
        return False

    def execute(self, userdata):
        pose = userdata.pose_in
        userdata.pose_out = pose
        obj = userdata.object_in
        userdata.object_out = obj

        param_obj = rospy.get_param(obj['name'])
        hole = param_obj['hole']

        # param_pose = rospy.get_param(pose)
        side = get_pickup_side(pose['name'])
        # param_pose['side']

        # best = self.getBestObject(objects)

        goal = OdomFineAdjustGoal()
        if side == "left":
            goal.target.x = -0.04
        else:
            goal.target.y = -0.05
        # goal.threshold.x = 0.05
        #goal.threshold.y = 0.05
        #goal.threshold.theta = 1 # this doesn't matter
        goal.max_dist = 0.6
        goal.duration = 11

        if not NO_FINE_ADJUST:
            self.odom_fine_client.send_goal(goal)

        if NO_VISION:
            self.current_hole = PoseStampedLabeled()
            self.current_hole.label = hole
            print "NO VISION JUST ACCEPT HOLE"
            rospy.sleep(1)

        else:
            self.is_active = True
            r = rospy.Rate(20)

            while not self.odom_fine_client.get_state() == GoalStatus.SUCCEEDED and not self.acceptHole(hole):
                # move
                r.sleep()

            if self.odom_fine_client.get_state() == GoalStatus.SUCCEEDED:
                print "MOVED"
                self.is_active = False
                #userdata.pose_out = "D0"
                # TODO set to normal place after done
                return "nothing_found"

        # stop

        for i in xrange(5):
            self.odom_fine_client.cancel_all_goals()
            # res = self.odom_fine_client.get_result()
        rospy.sleep(1.0)
        self.is_active = False
        if self.current_hole.label in ['M30'] and self.current_object.label in ['M20']:
            self.current_hole.label = 'M20'

        if not self.current_hole.label == hole:
            return "failed"

        #userdata.dist_out = res.dist
        userdata.point_out = self.current_hole.pose.pose

        return "success"


# class DetectObjectsState(smach.State):
# def __init__(self):
# smach.State.__init__(self, outcomes=['success', 'failed'], input_keys=['pose_in', 'object_in'], output_keys=['pose_out', 'object_out', 'point_out'])
# init_globals()

#         self.detect_pose = None    
#         rospy.Subscriber("/vision/detected_object", PoseStampedLabeled, self.cb_detect)

#     def cb_detect(self,msg):
#         self.detect_pose = msg

#     def execute(self, userdata):
#         rate = rospy.Rate(10)
#         for i in xrange(10): 
#             if (rospy.Time.now() - self.detect_pose.header.stamp).to_sec() < TIME_DIFF :
#                 userdata.pose_out = userdata.pose_in
#                 userdata.object_out = userdata.object_in

#                 userdata.point_out = self.detect_pose.pose
#                 return 'success'
#         return 'failed'




# class KinectDetectionState(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['success', 'failed'], input_keys=['pose_in'], output_keys=['pose_out', 'object_out', 'point_out'])
#         init_globals()

#         self.tfListen = tf.TransformListener()
#         #listen for some time
#         self.classes = rospy.get_param("classes")
#         self.objectLocations = {}


#         self.targetArray = rospy.Subscriber('/vision/targets', TargetArray, self.cbTargetArray)
#         self.copying = False
#         rospy.sleep(1)



#     def cbTargetArray(self, msg):
#         #print msg
#         for t in msg.targets:
#             pose_in = PoseStamped()
#             pose_in.pose = t.pose
#             pose_in.header.frame_id = msg.header.frame_id
#             obj_name = self.classes[t.identifier]


#             if self.tfListen.frameExists(pose_in.header.frame_id) and self.tfListen.frameExists(base_frame):
#                 time = self.tfListen.getLatestCommonTime(pose_in.header.frame_id, base_frame)
#                 pose_in.header.stamp = time
#                 pose = self.tfListen.transformPose(base_frame, pose_in)

#                 #while not self.copying:
#                 self.objectLocations[obj_name] = {}
#                 self.objectLocations[obj_name]['pose'] = pose
#                 self.objectLocations[obj_name]['confidence'] = t.confidence
#                 self.objectLocations[obj_name]['time'] = time
#                     #rospy.sleep(0.0001)


#     def getBestObject(self, objects):
#         self.copying = True
#         tmpDict = copy.deepcopy(self.objectLocations)
#         print tmpDict

#         self.copying = False
#         best = None
#         conf = 0
#         now = rospy.Time.now()
#         for obj in objects:
#             if obj in tmpDict.keys():
#                 if tmpDict[obj['name']]['confidence']>conf and  (now-tmpDict[obj['name']]['time']).to_sec() < TIME_DIFF:
#                     conf = tmpDict[obj['name']]['confidence']>conf
#                     best = obj
#         return best

#     def execute(self, userdata):
#         pose = userdata.pose_in
#         pos = rospy.get_param(pose)
#         userdata.pose_out = pose

#         objects = pos['objects']

#         best = self.getBestObject(objects)

#         print best

#         if best is None:
#             return 'failed'

#         object_name = best['name']
#         print self.objectLocations
#         userdata.point_out = self.objectLocations[object_name]['pose']
#         userdata.object_out = object_name

#         return 'success'


# class ScanForObjectsState(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['success', 'failed', 'nothing_found'], input_keys=['pose_in', 'dist_in'], output_keys=['pose_out', 'object_out', 'point_out', 'dist_out'])
#         init_globals()
#         #self.tfListen = tf.TransformListener()
#         #listen for some time
#         #self.classes = rospy.get_param("classes")
#         #self.objectLocations = {}

#         self.odom_fine_client = actionlib.SimpleActionClient('odom_move', OdomFineAdjustAction)

#         if not NO_FINE_ADJUST:
#             self.odom_fine_client.wait_for_server()

#         self.current_object = None
#         self.current_hole = None

#         rospy.Subscriber('/vision/detected_object', PoseStampedLabeled, self.cb_detect)
#         rospy.Subscriber('/vision/detected_hole', PoseStampedLabeled, self.cb_detect_hole)

#         #self.copying = False
#         #rospy.sleep(1)
#         self.counter = 0
#         self.running = False
#         self.processing = False

#     def cb_detect(self, msg):
#         if self.running and not self.processing:
#             self.current_object = msg

#     def cb_detect_hole(self, msg):
#         if self.running and not self.processing:
#             self.current_hole = msg

#     def dist(self, a, b):
#         return math.sqrt(pow(a.x - b.x, 2) + pow(a.y-b.y,2))


#     def acceptObject(self, objects):
#         self.processing = True
#         now = rospy.Time.now()
#         if self.current_object is None or (now-self.current_object.pose.header.stamp).to_sec() > 0.5:
#             self.processing = False
#             return False
#         found = False

#         if self.current_hole is not None and (now-self.current_hole.pose.header.stamp).to_sec() > 0.2:
#             if self.dist(self.current_object.pose.pose.position, self.current_hole.pose.pose.position) < 0.02:

#                 if self.current_object.label in ['F20_20_B']:
#                     if self.current_hole.label in ['RV20']:
#                         self.current_object.label = 'V20'


#                 if self.current_object.label in ['M20']:
#                     if self.current_hole.label in ['RV20']:
#                         self.current_object.label = 'V20'

#         if self.current_object.label in objects:
#             #check hole detector
#             #if hole detector gives rv20 override
#             #self.current_object.label = rv20
#             found = True

#         elif self.current_object.label in ['V20', 'R20']:
#             if 'V20' in objects or 'R20' in objects:
#                 found = True

#         if not found:
#             self.processing = False
#             return False

#         if self.current_object.pose.pose.position.y < 0.1 and self.current_object.pose.pose.position.y > -0.05:
#             self.counter += 1
#             if self.counter > 20:
#                 self.processing = False
#                 return True
#         self.processing = False
#         return False

#     def execute(self, userdata):
#         pose = userdata.pose_in
#         userdata.pose_out = pose
#         self.counter = 0
#         self.running = True
#         param_pose = rospy.get_param(pose['name'])
#         param_objects = param_pose['objects']

#         #side = param_pose['side']
#         side = get_pickup_side(pose['name'])

#         objects = [x['name'] for x in param_objects]
#         print "OBJECTS AT LOCATION :", objects
#         #best = self.getBestObject(objects)

#         goal = OdomFineAdjustGoal()
#         if side == "left":
#             goal.target.x = -0.05
#         else:
#             goal.target.y = -0.05
#         #goal.threshold.x = 0.05
#         #goal.threshold.y = 0.05
#         #goal.threshold.theta = 1 # this doesn't matter
#         dist_driven = userdata.dist_in
#         goal.max_dist = 0.65 - dist_driven
#         goal.duration = 11

#         if not NO_FINE_ADJUST:
#             self.odom_fine_client.send_goal(goal)


#         if NO_VISION:
#             self.current_object = PoseStampedLabeled()
#             self.current_object.label = objects[0]
#             print "NO VISION JUST TAKE FIRST OBJECT"
#             rospy.sleep(1.0)


#         else:
#             r = rospy.Rate(20)



#             while not self.odom_fine_client.get_state() == GoalStatus.SUCCEEDED and not self.acceptObject(objects):
#             # move
#                 r.sleep()

#             if self.odom_fine_client.get_state() == GoalStatus.SUCCEEDED:
#                 print "MOVED"
#                 #userdata.pose_out = "D0"
#                 userdata.dist_out = 0
#                 self.running = False
#                 return "nothing_found"

#         label = self.current_object.label
#         for i in xrange(5):
#             self.odom_fine_client.cancel_all_goals()
#         rospy.sleep(1)

#         # stop
#         self.running = False
#         self.current_object.label = label

#         ##HACK for V20 and R20 plastic
#         if self.current_object.label in ['V20', 'R20']:
#             if 'V20' in objects:
#                 self.current_object.label = 'V20'
#             if 'R20' in objects:
#                 self.current_object.label = 'R20'

#         if not self.current_object.label in objects:
#             self.running = False
#             return "failed"

#         userdata.point_out = self.current_object.pose.pose

#         ## Find param object with place position and type
#         object_out = {}
#         for obj in param_objects:
#             if self.current_object.label == obj['name']:
#                 object_out = obj
#                 break

#         userdata.object_out = object_out
#         return "success"




