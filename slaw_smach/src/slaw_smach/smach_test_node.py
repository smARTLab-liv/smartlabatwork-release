#!/usr/bin/env python


import rospy
from ArmStates import *
from MoveStates import AlignToPlatformState, MoveBack, AlignToObjectState, DestinationPlaceManager, MoveBackWithAlign
from arm_helpers import *
from ObjectDetectState import SwitchObjectHoleManagerOnState, SwitchObjectHoleManagerOffState, VerifyObjectState, \
    VerifyHoleState
from DecisionStates import GetNextObjectLocationState
from std_srvs.srv import Empty
import sys
import select
import numpy as np
from slaw_srvs.srv import RemoveObjectAtLocationRequest, RemoveObjectAtLocation
from geometry_msgs.msg import PoseStamped

arm_up_pose = [0.011, 1.04883, -2.43523, 1.73184, 0.2]

GRIP_X = 0.25


class SmachTestNode:
    def __init__(self):
        self.userdata = lambda: 0
        self.userdata.pose_in = {'name': 'left'}
        self.userdata.pose_out = {'name': 'left'}
        self.arm_base_link_height = rospy.get_param('arm_base_link_height')
        self.joints = None
        self.mount_offset = rospy.get_param("arm_rot_offset")

        self.delete_all_objects_from_location = rospy.ServiceProxy('/vision/object_manager/remove_all_objects',
                                                                   RemoveObjectAtLocation)
        self.delete_object_from_location = rospy.ServiceProxy('/vision/object_manager/remove_object',
                                                              RemoveObjectAtLocation)

        # init states
        self.grip = Grip()
        self.fine_place = Place()
        self.pre_grip = PreGrip()
        self.pre_cbt = PreGripCBT()
        self.tuck = TuckArm()
        self.tuck_drive = TuckForDrive()
        self.switch_vision_on = SwitchObjectHoleManagerOnState()
        self.switch_vision_off = SwitchObjectHoleManagerOffState()
        self.align_platform = AlignToPlatformState()
        self.verify_object = VerifyObjectState()
        self.move_back = MoveBackWithAlign(0.5, 0.075)
        self.move_forward = MoveBack(0.5, -0.1)
        self.get_next_object = GetNextObjectLocationState()
        self.align_to_object = AlignToObjectState()
        self.place_on_platform = PlaceOnPlatform()
        self.pick_from_platform = PickFromPlatform()
        self.destination_manager = DestinationPlaceManager()
        self.verify_hole = VerifyHoleState()
        self.tb_arm = ArmForWait()
        print 'All initialized'

    def testGrip(self, offset):
        # self.gripperClose(False)
        # self.gripperClose(True)
        #rospy.sleep(2)
        side = self.userdata.pose_in['name']
        height = TABLE_HEIGHT + ABOVE_TABLE - self.arm_base_link_height
        point = [GRIP_X, offset, height]  #0.28 #0.15
        conf = np.array(call_ik_solver(point, side=side))
        point = [GRIP_X, offset, 0.0]  #0.28 #0.15

        conf2 = np.array(call_ik_solver(point, side=side))
        #    conf2[3] = conf[3]#

        go([conf2, conf])

        gripper_close(False)
        go([conf2])


    def switchOffMotors(self):
        rospy.wait_for_service('/arm_1/switchOffMotors')
        switch_off = rospy.ServiceProxy('/arm_1/switchOffMotors', Empty)
        try:
            switch_off()
            print 'switch off motors'
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


    def armUp(self):
        arm_up_without_turn = [[x for x in arm_up_pose]]
        arm_up_without_turn[0][0] = configuration[0]
        arm_up_without_turn[0][4] = configuration[4]
        go(arm_up_without_turn)

    def processKey(self, key):
        try:
            if key == 'left\n':
                self.userdata.pose_in = {'name': 'left'}
                self.userdata.pose_out = {'name': 'left'}
            if key == 'front\n':
                self.userdata.pose_in = {'name': 'front'}
                self.userdata.pose_out = {'name': 'front'}

            if key == 'joints\n':
                print configuration
                self.joints = [x for x in configuration]
            if key == 'play-joints\n':
                go([self.joints])
            if key == 'tb-arm\n':
                print self.tb_arm.execute(self.userdata)
            if key == 'back\n':
                print self.move_back.execute(self.userdata)
            if key == 'forward\n':
                print self.move_forward.execute(self.userdata)

            if key == 'screw-ppt\n':
                print self.verify_hole.execute(self.userdata)
                self.userdata.point_in = self.userdata.point_out
                self.userdata.object_in = self.userdata.object_out
                self.userdata.backplate_pose_in = self.userdata.backplate_pose_out
                print self.pick_from_platform.execute(self.userdata)
                print self.fine_place.execute(self.userdata)
            if key == '+\n':
                turn = [x for x in configuration]
                turn[0] += 0.025
                go([turn])
            if key == '-\n':
                turn = [x for x in configuration]
                turn[0] -= 0.025
                go([turn])
            if key == 'calibrate-left\n':
                self.testGrip(-0.10)
            if key == 'calibrate-right\n':
                self.testGrip(0.10)
            if key == 'calibrate-center\n':
                self.testGrip(-0.00)
            if key == 'align\n':
                print self.align_platform.execute(self.userdata)

            if key == 'place\n':
                print self.place.execute(self.userdata)
            if key == 'tuck\n':
                print self.tuck.execute(self.userdata)
            if key == 'open\n':
                gripper_close(False)
            if key == 'close\n':
                gripper_close(True)
            if key == 'off\n':
                self.switchOffMotors()

            if key == 'cam-test\n':
                go([cam_test])

            if key == 'pre-grip\n':
                print self.pre_grip.execute(self.userdata)
            if key == 'pre-cbt\n':
                print self.pre_cbt.execute(self.userdata)

            if key == 'grip-detect\n':
                print self.grip.execute(self.userdata)
            if key == 'ppt\n':
                print self.fine_place.execute(self.userdata)

            if key == 'place-back\n':
                print self.place_on_platform.execute(self.userdata)
            if key == 'get-back\n':
                self.destination_manager.reset(None)
                print self.destination_manager.execute(self.userdata)
                self.userdata.backplate_pose_in = self.userdata.backplate_pose_out
                self.userdata.object_in = self.userdata.object_out
                self.userdata.point_out = self.userdata.point_out
                print self.pick_from_platform.execute(self.userdata)

            if key == 'scan\n':
                print self.align_platform.execute(self.userdata)
                self.delete_all_objects_from_location(loc=self.userdata.pose_in['name'])

                print self.switch_vision_on.execute(self.userdata)
                rospy.sleep(1)
                print self.move_back.execute(self.userdata)
                print self.switch_vision_off.execute(self.userdata)
                print self.get_next_object.execute(self.userdata)

                self.userdata.point_in = self.userdata.point_out
                print self.userdata.point_out
                print self.align_to_object.execute(self.userdata)

                print self.verify_object.execute(self.userdata)
                self.userdata.object_in = self.userdata.object_out
                self.userdata.point_in = self.userdata.point_out

                print self.grip.execute(self.userdata)

                self.testGrip(-0.10)
                print self.pre_grip.execute(self.userdata)

                pose_test = PoseStamped()
                pose_test.header.stamp = rospy.Time()
                pose_test.header.frame_id = '/arm_base_link'
                pose_test.pose = self.userdata.point_out
                self.delete_object_from_location(loc=self.userdata.pose_in['name'], pose=pose_test)



                # #second time
                print self.get_next_object.execute(self.userdata)

                self.userdata.point_in = self.userdata.point_out
                print self.userdata.point_out

                print self.align_to_object.execute(self.userdata)

                print self.verify_object.execute(self.userdata)
                self.userdata.object_in = self.userdata.object_out
                self.userdata.point_in = self.userdata.point_out

                print self.grip.execute(self.userdata)

            if key == 'play-joints\n':
                if self.joints is None:
                    print "no joints set yet"
                    return
                go([self.joints])

            if key == 'detect\n':
                self.delete_all_objects_from_location(loc=self.userdata.pose_in['name'])
                # self.switch_vision_off.execute(self.userdata)
                print self.verify_object.execute(self.userdata)
                self.userdata.object_in = self.userdata.object_out
                self.userdata.point_in = self.userdata.point_out
                self.userdata.backplate_pose_in = self.userdata.backplate_pose_out
                # print self.userdata
                #if key == 'start\n':
                #self.switch_vision_on.execute(self.userdata)
            if key == 'up\n':
                self.armUp()
            if key == 'nippel1\n':
                confs = confs_for_platform_nipple(1, 0, 0)
                go(confs)
            if key == 'nippel2\n':
                confs = confs_for_platform_nipple(2, 0, 0)
                go(confs)
            if key == 'big_hole\n':
                confs = confs_for_platform_big_hole(0, 0)
                go(confs)
        except Exception as e:
            print "did not work", e
        print 'done'


def main():
    rospy.init_node("TestNode")
    rospy.sleep(0.001)  # wait for time
    smach_node = SmachTestNode()
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        i, o, e = select.select([sys.stdin], [], [], 0.0001)
        for s in i:
            if s == sys.stdin:
                input_cmd = sys.stdin.readline()
                smach_node.processKey(input_cmd)
        rate.sleep()


if __name__ == '__main__':
    main()
    
