#!/usr/bin/env python
import math
import numpy as np

import rospy
import tf
import smach
from slaw_srvs.srv import *

from arm_helpers import call_ik_solver, init_arm_helpers, grip_point, NO_ARM, get_angle, \
    PRE_PLACE_HEIGHT, rotate_only, go, gripper_close, configuration, cam_test_drop, cam_test, \
    PRE_GRIP_HEIGHT, STEP_SIZE, pre_grip, MAX_TRIES, pre_kinect, get_pick_height_for_obj, get_place_height_for_obj, \
    place_on_platform, pick_from_platform
from smach_helpers import get_pickup_side, dict_to_backplate_object_msg, publish_status

joints = rospy.get_param("joints")


class OpenGripper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        init_arm_helpers()

    def execute(self, userdata):
        gripper_close(False)
        return 'done'


class Grip(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'too_far', 'failed', 'failed_after_grip'],
                             input_keys=['pose_in', 'point_in', 'object_in'], output_keys=['object_out'])
        init_arm_helpers()

    def execute(self, userdata):
        obj = userdata.object_in

        # offset detected by object detection
        offset = userdata.point_in
        pose = userdata.pose_in
        # to which side
        side = get_pickup_side(pose['name'])

        quat = [offset.orientation.x, offset.orientation.y, offset.orientation.z, offset.orientation.w]
        r, p, y = tf.transformations.euler_from_quaternion(quat)

        new_grip_point = [x for x in grip_point]
        new_grip_point[0] += offset.position.x
        new_grip_point[1] -= offset.position.y
        new_grip_point[2] += get_pick_height_for_obj(obj['label'])

        publish_status('I am grasping object ' + str(obj['name']) + '.')

        ## TODO check HEIGHT FOR HIGH OBJECTS

        if NO_ARM:
            return 'success'

        conf = None
        try:
            conf = call_ik_solver(new_grip_point, side=side)
        except:
            print "got no config from IK Solver"
        if len(conf) == 0:
            return 'too_far'
        else:
            conf = np.array(conf)

        grip_angle = get_angle(conf[4], y)
        straight_angle = conf[4]
        conf[4] = grip_angle
        height = new_grip_point[2]

        new_grip_point[2] += PRE_GRIP_HEIGHT

        confs = []
        rotate = rotate_only(side=side)

        confs.append(rotate)
        first = None
        gripX = new_grip_point[0]
        gripY = new_grip_point[1]

        while new_grip_point[2] > height:

            new_grip_point[2] -= STEP_SIZE
            new_grip_point[2] = max(height, new_grip_point[2])
            new_grip_point[0] = gripX
            new_grip_point[1] = gripY
            conf2 = []

            while len(conf2) == 0:
                try:
                    print 'get conf', new_grip_point
                    conf2 = call_ik_solver(new_grip_point, side=side)
                    print conf2
                except:
                    #new_grip_point[0] -= 0.01 #get closer to solve pre-grip
                    print 'got not config'
                    return 'too_far'
                if len(conf2) > 0:
                    conf2 = np.array(conf2)
                    print conf2
                new_grip_point[0] -= 0.005  #get closer to solve pre-grip
            conf2[4] = conf[4]
            if first is None:
                first = [x for x in conf2]
            confs.append(conf2)

        gripper_close(False)
        confs.append(conf)

        if not go(confs):
            return 'failed'

        if userdata.object_in['label'] == 'M20_100_h':
            obj = userdata.object_in
            ### TODO set correct rotation
            obj['rotation'] = -y - (grip_angle - straight_angle)
            userdata.object_out = obj
            print "rotation is", obj['rotation'], y, grip_angle - straight_angle
            gripper_close(True, heavy=True)
        else:
            gripper_close(True)

        if not go([first]):
            return 'failed_after_grip'
        return 'success'


class Place(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'too_far', 'failed', 'failed_after_place'],
                             input_keys=['pose_in', 'point_in', 'object_in'])
        init_arm_helpers()

    def get_rotation(self, h_yaw, straight, s_yaw):
        s_yaw = abs(s_yaw)

        if h_yaw < 0:
            h_yaw += 2. * math.pi
        dif = h_yaw - s_yaw
        if dif > math.pi:
            dif -= 2. * math.pi
        ang = straight - dif
        print ang, straight, dif
        if joints['arm_joint_5']['max'] > ang > joints['arm_joint_5']['min']:
            return ang
        return None

    def execute(self, userdata):
        # offset detected by object detection
        offset = userdata.point_in
        pose = userdata.pose_in
        obj = userdata.object_in

        side = get_pickup_side(pose['name'])
        # to which side
        publish_status('I am placing the object <br> on the service area.')
        quat = [offset.orientation.x, offset.orientation.y, offset.orientation.z, offset.orientation.w]
        r, p, y = tf.transformations.euler_from_quaternion(quat)

        new_grip_point = [x for x in grip_point]
        new_grip_point[0] += offset.position.x
        new_grip_point[1] -= offset.position.y
        new_grip_point[2] += get_place_height_for_obj(obj['label'])

        if NO_ARM:
            return 'success'

        conf = None
        try:
            conf = call_ik_solver(new_grip_point, side=side)
        except:
            print "got no config from IK Solver"

        if len(conf) == 0:
            return 'too_far'
        else:
            conf = np.array(conf)

        straight = conf[4]
        grip_angle = get_angle(conf[4], y)
        conf[4] = grip_angle
        new_grip_point[2] += PRE_PLACE_HEIGHT

        rotate = rotate_only(side=side)

        gripX = new_grip_point[0]
        gripY = new_grip_point[1]

        new_grip_point[0] = gripX
        new_grip_point[1] = gripY
        conf2 = []

        while len(conf2) == 0:
            try:
                print 'get conf', new_grip_point
                conf2 = call_ik_solver(new_grip_point, side=side)
                print conf2
            except:
                # new_grip_point[0] -= 0.01 #get closer to solve pre-grip
                print 'got not config'
                return 'too_far'
            if len(conf2) > 0:
                conf2 = np.array(conf2)
                new_grip_point[0] -= 0.005  #get closer to solve pre-grip
        conf2[4] = conf[4]

        go([rotate])

        print obj

        if obj['label'] == 'M20_100_h' and obj['type'] == 'PPT':
            m20_rot = obj['rotation']
            yaw = self.get_rotation(y, straight, m20_rot)
            if yaw is None:
                print "rotate screw!"
                yaw = self.get_rotation(y, straight, math.pi - abs(m20_rot))
                temp_y = get_angle(straight, y - math.pi / 2.)
                scnd_tmp = temp_y + math.pi
                if scnd_tmp > joints['arm_joint_5']['max']:
                    scnd_tmp -= 2. * math.pi
                conf[4] = temp_y
                conf2[4] = temp_y
                go([conf2, conf])
                gripper_close(False)
                go([conf2])
                conf[4] = scnd_tmp
                conf2[4] = scnd_tmp
                go([conf2, conf])
                gripper_close(True, heavy=True)
                go([conf2])
            conf[4] = yaw
            conf2[4] = yaw

        if not go([conf2, conf]):
            return 'failed'

        gripper_close(False)

        #rotate??
        if False:
            rotate_wiggle = [x for x in conf]
            rotate_wiggle[4] -= 0.1

            rotate_wiggle2 = [x for x in conf]
            rotate_wiggle2[4] += 0.1

            if not go([rotate_wiggle, rotate_wiggle2, conf]):
                return 'failed'

        if not go([conf2]):
            return 'failed_after_place'
        return 'success'


class PickFromPlatform(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'], input_keys=['backplate_pose_in', 'object_in'])
        init_arm_helpers()
        self.remove_from_platform = rospy.ServiceProxy('/backplate_manager/remove_object_from_plate',
                                                       BackplateStateModify)

    def execute(self, userdata):
        backplate_pose = userdata.backplate_pose_in
        obj = userdata.object_in

        pick_from_platform(obj, backplate_pose)
        publish_status('I am getting the ' + obj['name'] + '<br> from my back plate.')
        obj['backplate_location'] = backplate_pose
        obj_msg = dict_to_backplate_object_msg(obj)
        self.remove_from_platform(obj=obj_msg)
        return 'success'


class PlaceOnPlatform(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'], input_keys=['backplate_pose_in', 'object_in'])
        init_arm_helpers()
        self.add_to_platform = rospy.ServiceProxy('/backplate_manager/add_object_to_plate', BackplateStateModify)

    def execute(self, userdata):
        backplate_pose = userdata.backplate_pose_in
        obj = userdata.object_in

        place_on_platform(obj, backplate_pose)
        publish_status('I am placing ' + obj['name'] + '<br> on my back plate.')
        obj['backplate_location'] = backplate_pose
        obj_msg = dict_to_backplate_object_msg(obj)
        self.add_to_platform(obj_msg)
        return 'success'


class TuckForDrive(smach.State):
    def __init__(self, blocking=False, no_rotate=False):
        smach.State.__init__(self, outcomes=['done'], input_keys=['pose_in'])
        init_arm_helpers()
        self.blocking = blocking
        self.no_rotate = no_rotate

    def execute(self, userdata):
        confs = []
        if not self.no_rotate:
            pose = userdata.pose_in
            side = get_pickup_side(pose['name'])

            # print joints
            tucked = rotate_only(side='left')
            #tucked[0] = configuration[0]
            #tucked[1] = joints['arm_joint_2']['min']
            #tucked[2] = joints['arm_joint_3']['max']
            #tucked[3] = joints['arm_joint_4']['min']
            #tucked[4] = configuration[4]
            confs.append(tucked)

        tucked_for_drive = rotate_only(side='left')
        # tucked[0] = conf[0]
        tucked_for_drive[1] = joints['arm_joint_2']['min']
        tucked_for_drive[2] = joints['arm_joint_3']['max']
        tucked_for_drive[3] = joints['arm_joint_4']['min']
        tucked_for_drive[4] = joints['arm_joint_5']['straight']
        confs.append(tucked_for_drive)
        go(confs, blocking=self.blocking)
        return 'done'


class RV20Trash(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        init_arm_helpers()

    def execute(self, userdata):
        # locations = rosparam.get('locations')
        # pose = locations[0]
        #pre_grip[0] = rotate[0]
        go([cam_test_drop])
        gripper_close(False)
        return 'done'


class RV20ReplaceObjectRotate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed'], input_keys=['pose_in'])
        init_arm_helpers()

    def execute(self, userdata):
        # locations = rosparam.get('locations')
        # pose = locations[0]
        pose = userdata.pose_in
        side = get_pickup_side(pose['name'])
        rotate = rotate_only(side)
        #pre_grip[0] = rotate[0]

        #up = [x for x in cam_test]
        #up[0] = configuration[0]
        if go([rotate]):
            return 'success'
        else:
            return 'failed'


class RV20ReplaceUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        init_arm_helpers()

    def execute(self, userdata):
        # locations = rosparam.get('locations')
        # pose = locations[0]
        up = [x for x in cam_test]
        up[0] = configuration[0]

        go([up])
        return 'done'


class RV20CheckArmPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        init_arm_helpers()

    def execute(self, userdate):
        up = [x for x in cam_test]
        up[0] = configuration[0]
        go([cam_test])
        return 'done'



class GripCBT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['end'])
        init_arm_helpers()

    def execute(self, userdata):
        rospy.sleep(4)
        gripper_close(True)
        tucked = [x for x in configuration]
        # tucked[0] = configuration[0]
        tucked[1] = joints['arm_joint_2']['min']
        tucked[2] = joints['arm_joint_3']['max']
        tucked[3] = joints['arm_joint_4']['min']
        # tucked[4] = configuration[4]

        tucked_for_drive = rotate_only(side='left')
        #tucked[0] = conf[0]
        tucked_for_drive[1] = joints['arm_joint_2']['min']
        tucked_for_drive[2] = joints['arm_joint_3']['max']
        tucked_for_drive[3] = joints['arm_joint_4']['min']
        tucked_for_drive[4] = joints['arm_joint_5']['straight']

        go([tucked, tucked_for_drive])

        return 'end'


class ArmForWait(smach.State):
    def __init__(self, blocking=True):
        smach.State.__init__(self, outcomes=['success'])
        init_arm_helpers()
        self.blocking = blocking

    def execute(self, userdata):
        side = 'front'
        rotate = rotate_only(side)
        conf = [x for x in pre_grip]
        conf[0] = rotate[0] + 0.2
        conf[3] -= 0.3
        conf[1] = joints['arm_joint_2']['straight']
        # gripperClose(False)
        # rotate,
        go([conf], blocking=self.blocking)
        return 'success'


class PlaceOnTurtlebot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'], input_keys=['tb_pose_in'])

    def execute(self, ud):
        position = ud.tb_pose_in.position
        p = np.array([position.x, position.y])
        dist = 0.28 + 0.07
        norm = p / np.linalg.norm(p)
        p -= dist * norm
        point = [p[0], -p[1], 0]

        conf = call_ik_solver(point, side='front')
        while len(conf) == 0:
            p -= 0.01 * norm
            point = [p[0], -p[1], 0]
            conf = call_ik_solver(point, side='front')
            print p

        print conf
        conf = np.array(conf)
        conf[0] += 0.2
        conf[3] -= math.pi / 2.
        conf[4] = joints['arm_joint_5']['straight']
        go([conf], blocking=True)
        rospy.sleep(1.0)
        gripper_close(False)

        return 'done'


class PreGrip(smach.State):
    def __init__(self, blocking=True):
        smach.State.__init__(self, outcomes=['success'], input_keys=['pose_in'])
        init_arm_helpers()
        self.blocking = blocking

    def execute(self, userdata):
        pose = userdata.pose_in
        # position = rospy.get_param(pose)
        # print 'go', pose

        side = get_pickup_side(pose['name'])
        #print 'go'

        rotate = rotate_only(side)
        pre_grip[0] = rotate[0]
        # gripperClose(False)
        # rotate,
        go([pre_grip], blocking=self.blocking)
        return 'success'


class PreGripCBT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed'], input_keys=['pose_in'], output_keys=['pose_out'])
        init_arm_helpers()

    def execute(self, userdata):
        pose = userdata.pose_in

        # position = rospy.get_param(pose)
        userdata.pose_out = pose

        side = get_pickup_side(pose['name'])

        rotate = rotate_only(side)
        # pre_grip[0] = rotate[0]
        gripper_close(False)

        new_grip_point = [x for x in grip_point]
        new_grip_point[0] += 0.05
        new_grip_point[2] += 0.015
        conf = np.array(call_ik_solver(new_grip_point, side=side))
        conf[4] = get_angle(conf[4], math.pi / 2)
        if go([rotate, conf]):
            return 'success'
        else:
            return 'failed'


class RotateToPlatform(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'], input_keys=['pose_in'])
        init_arm_helpers()

    def execute(self, userdata):
        pose = userdata.pose_in
        side = get_pickup_side(pose['name'])

        rotate = rotate_only(side=side, turned=False)
        go([rotate])
        return 'done'


class TuckArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'not_reached', 'failed'])
        init_arm_helpers()
        self.counter = 0

    def execute(self, userdata):

        # side = position['side']
        rotate = rotate_only('left', turned=False)
        arm_up = [rotate[0], joints['arm_joint_2']['straight'], joints['arm_joint_3']['straight'],
                  joints['arm_joint_4']['straight'], rotate[4]]

        tuck = [rotate[0], joints['arm_joint_2']['min'], joints['arm_joint_3']['max'], joints['arm_joint_4']['min'],
                rotate[4]]

        # print arm_up, tuck
        #return 'success'
        if go([rotate, arm_up, tuck]):
            self.counter = 0
            return 'success'
        else:
            if self.counter < MAX_TRIES:
                self.counter += 1
                return 'not_reached'
            else:
                self.counter = 0
                return 'failed'


class ArmUp(smach.State):
    def __init__(self, blocking=True):
        smach.State.__init__(self, outcomes=['done'])
        init_arm_helpers()
        self.blocking = blocking

    def execute(self, userdata):
        arm_up = [configuration[0], joints['arm_joint_2']['straight'], joints['arm_joint_3']['straight'],
                  joints['arm_joint_4']['straight'], configuration[4]]
        go([arm_up], blocking=self.blocking)
        return 'done'


class PreKinect(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed'], input_keys=['pose_in'], output_keys=['pose_out'])
        init_arm_helpers()

    def execute(self, userdata):
        pose = userdata.pose_in
        position = rospy.get_param(pose)
        side = position['side']
        rotate = rotate_only(side, turned=True)
        pre_kinect[0] = rotate[0]
        userdata.pose_out = pose
        if go([rotate, pre_kinect]):
            return 'success'
        else:
            return 'failed'



