#!/usr/bin/env python

import rospy
import smach
from slaw_srvs.srv import GetObjectAtLocation, GetObjectAtLocationRequest, BackplateStateStatus, RemoveObjectAtLocation, \
    GetDist
from geometry_msgs.msg import PoseStamped
from smach_helpers import get_objects_on_plate, init_helpers, get_pickup_side, publish_status
from MoveStates import DISTANCE_TO_PLATFORM


class GetNextObjectLocationState(smach.State):
    def __init__(self, holes=False):
        smach.State.__init__(self, outcomes=['success', 'nothing_found', 'nothing_left', 'backplate_full'], input_keys=['pose_in'],
                             output_keys=['point_out'])
        init_helpers()
        self.holes = holes
        self.srv_string = ''
        if not holes:
            self.srv_string = "/vision/object_manager/get_best"
        else:
            self.srv_string = "/vision/hole_manager/get_best"
        self.get_best_at_location = rospy.ServiceProxy(self.srv_string, GetObjectAtLocation)

    def execute(self, userdata):
        pose = userdata.pose_in
        side = get_pickup_side(pose['name'])
        if not self.holes:
            objects_on_plate = get_objects_on_plate()
            if len(objects_on_plate) == 3:
                return 'backplate_full'

            param_pose = rospy.get_param(pose['name'])
            param_objects = param_pose['objects']
            objects = [x['name'] for x in param_objects]
            print "OBJECTS AT LOCATION :", objects

        else:
            objects_on_plate = get_objects_on_plate()

            objects = []
            for obj in objects_on_plate:
                if obj.destination_loc == pose['name']:
                    param_obj = rospy.get_param(obj.label)
                    hole = param_obj['hole']
                    objects.append(hole)
                else:
                    rospy.logerr('object not for this platform' + str(obj.label))
            # objects = [x['name'] for x in param_objects]
            print "HOLES TO FIND  AT LOCATION :", objects

        if len(objects) == 0:
            return 'nothing_left'

        best = None
        req = GetObjectAtLocationRequest()
        req.loc = pose['name']
        req.side = side
        req.objects_to_pick = objects
        try:
            rospy.wait_for_service(self.srv_string)
            best = self.get_best_at_location(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return 'nothing_found'

        if best.object.label == 'None':
            print 'best.object.label == None'
            return 'nothing_found'
        userdata.point_out = best.object.pose.pose
        if self.holes:
            publish_status('I am driving to the <br> next hole (' + str(best.object.label) + ').')
        else:
            publish_status('I am driving to the <br> next object (' + str(best.object.label) + ').')

        return 'success'


class SleepState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'], input_keys=['pose_in'])

    def execute(self, userdata):
        # rospy.loginfo('Executing state "%s"'%self.label)
        pose = userdata.pose_in
        sleep = pose['sleep']
        rospy.sleep(sleep)
        return 'done'


class GetNextBNTLoc(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'], output_keys=['pose_out'])

    def execute(self, userdata):
        locations = rospy.get_param('locations', [])
        if len(locations) > 0:
            userdata.pose_out = locations[0]
            return 'done'
        else:
            pose = {'name': 'D0', 'type': 'End'}
            userdata.pose_out = pose
            return 'done'


class DeleteCurrentGoalState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        # rospy.loginfo('Executing state "%s"'%self.label)
        locations = rospy.get_param('locations')
        locations.pop(0)

        rospy.set_param('locations', locations)
        return 'done'


class DecideAfterMoveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['BNT', 'End', 'Pickup', 'Place'], input_keys=['pose_in'])


    def execute(self, userdata):
        pose = userdata.pose_in
        return pose['type']


class DecideBeforePreGripState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CBT', 'Pickup'], input_keys=['pose_in'])


    def execute(self, userdata):
        pose = userdata.pose_in
        return pose['type']


class DecideTest(smach.State):
    def __init__(self, min_distance=0.03):
        smach.State.__init__(self, outcomes=['BNT', 'Manipulation', 'NoPlan'])


    def execute(self, userdata):
        locations = rospy.get_param('locations', [])

        if len(locations) > 0:
            if locations[0]['type'] == 'BNT':
                return 'BNT'
            else:
                return 'Manipulation'

        return 'NoPlan`'


class ShouldAlign(smach.State):
    def __init__(self, min_distance=0.03):
        smach.State.__init__(self, outcomes=['align', 'continue'],
                             input_keys=['pose_in'])

        self.dist_platform = rospy.ServiceProxy("/vision/get_dist", GetDist)
        self.min_distance_to_platform = min_distance

    def execute(self, userdata):
        distance_to_platform = self.get_distance_to_platform(userdata.pose_in)
        print distance_to_platform
        if distance_to_platform is None:
            return 'continue'

        if distance_to_platform.dist <= self.min_distance_to_platform:
            return 'align'
        else:
            return 'continue'

    def get_distance_to_platform(self, pose):
        side = get_pickup_side(pose['name'])

        cur_dist = None
        try:
            cur_dist = self.dist_platform('side')
            # print poseLoc
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        return cur_dist


class BlacklistHole(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'],
                             input_keys=['point_in'])
        self.blacklist_object_from_location = rospy.ServiceProxy('/vision/hole_manager/blacklist_hole',
                                                                 RemoveObjectAtLocation)

    def execute(self, userdata):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time()
        pose_msg.header.frame_id = '/arm_base_link'
        pose_msg.pose = userdata.point_in
        self.blacklist_object_from_location(loc=userdata.pose_in['name'], pose=pose_msg)
        return 'done'


class BlacklistObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'],
                             input_keys=['point_in', 'pose_in'])
        self.blacklist_object_from_location = rospy.ServiceProxy('/vision/object_manager/blacklist_object',
                                                                 RemoveObjectAtLocation)

    def execute(self, userdata):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time()
        pose_msg.header.frame_id = '/arm_base_link'
        pose_msg.pose = userdata.point_in
        self.blacklist_object_from_location(loc=userdata.pose_in['name'], pose=pose_msg)
        return 'done'


class DeleteObjFromLoc(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue', 'nothing_left'],
                             input_keys=['object_in', 'pose_in', 'point_in'])
        self.delete_object_from_location = rospy.ServiceProxy('/vision/object_manager/remove_object',
                                                              RemoveObjectAtLocation)

    def execute(self, userdata):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time()
        pose_msg.header.frame_id = '/arm_base_link'
        pose_msg.pose = userdata.point_in

        self.delete_object_from_location(loc=userdata.pose_in['name'], pose=pose_msg)
        obj = userdata.object_in

        objects_list = rospy.get_param(userdata.pose_in['name'])
        for o in objects_list['objects']:
            if obj['name'] == o['name'] and obj['type'] == o['type'] \
                    and obj['destination_location'] == o['destination_location']:
                objects_list['objects'].remove(o)
                rospy.set_param(userdata.pose_in['name'], objects_list)
                break
        if len(objects_list['objects']) > 0:
            return 'continue'
        else:
            return 'nothing_left'


class DecideBeforePlaceState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['PPT', 'Normal', 'nothing_found'], input_keys=['pose_in'])


    def execute(self, userdata):
        pose = userdata.pose_in['name']
        objects_on_plate = get_objects_on_plate()
        for obj in objects_on_plate:
            if obj.destination_loc == pose:
                return obj.place_type

        return 'nothing_found'


class DecideRV20State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['RV20', 'Normal'], input_keys=['object_in'], output_keys=['pose_out'])

    def execute(self, userdata):
        obj = userdata.object_in

        if obj['name'] in ['V20', 'R20']:
            locations = rospy.get_param('locations')
            pose = locations[0]
            userdata.pose_out = pose
            return 'RV20'
        else:
            return 'Normal'

