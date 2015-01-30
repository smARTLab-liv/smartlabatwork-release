#!/usr/bin/env python

from ArmStates import *
from MoveStates import *
from ObjectDetectState import *
from DecisionStates import *
import smach_ros


class SetPose(smach.State):
    def __init__(self, state, type_in):
        self.type = type_in
        self.state = state
        smach.State.__init__(self, outcomes=['success'], output_keys=['pose_out'])

    def execute(self, userdata):
        pose = {'name': self.state, 'type': self.type}
        userdata.pose_out = pose
        return 'success'

class AddToBackplate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'], input_keys=['object_in'])
        self.add_to_platform = rospy.ServiceProxy('/backplate_manager/add_object_to_plate', BackplateStateModify)
        init_arm_helpers()
        
    def execute(self, userdata):
        obj = userdata.object_in
        publish_status('I am placing ' + obj['name'] + '<br> on my back plate.')
        obj['backplate_location'] = 'center_1'
        obj_msg = dict_to_backplate_object_msg(obj)
        self.add_to_platform(obj_msg)
        return 'success'

        
def ppt_place_from_gripper():
    sm = smach.StateMachine(outcomes=['done', 'arm_failed', 'should_not_happen'], input_keys=['pose'])
    with sm:
        smach.StateMachine.add('PreGrip_PPT', PreGrip(blocking=True),
                               transitions={'success': 'VerifyHole'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('VerifyHole', VerifyHoleState(),
                               transitions={'success': 'PlaceFromGripper', 'too_far': 'should_not_happen',
                                            'failed': 'should_not_happen'},
                               remapping={'pose_in': 'pose', 'point_out': 'point',
                                          'object_out': 'object', 'backplate_pose_out': 'backplate_pose'})
        smach.StateMachine.add('PlaceFromGripper', Place(),
                               transitions={'too_far': 'should_not_happen', 'success': 'PreGrip_after_place_PPT',
                                            'failed': 'RecoverArmToPreGrip',
                                            'failed_after_place': 'RecoverArmToPreGrip'},
                               remapping={'pose_in': 'pose', 'point_in': 'point', 'object_in': 'object'})
        smach.StateMachine.add('PreGrip_after_place_PPT', PreGrip(blocking=True),
                               transitions={'success': 'TuckForDrive'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('TuckForDrive', TuckForDrive(blocking=True),
                               transitions={'done': 'done'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('RecoverArmToPreGrip', recover_arm_to_pre_grip(),
                               transitions={'success': 'should_not_happen', 'failed': 'arm_failed'},
                               remapping={'pose_in': 'pose'})

    return sm


def pick_and_keep():
    sm = smach.StateMachine(outcomes=['done', 'arm_failed', 'not_implemented', 'should_not_happen'],
                            input_keys=['pose'])
    with sm:
        smach.StateMachine.add('PreGrip', PreGrip(blocking=True), transitions={'success': 'VerifyObject'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('VerifyObject', VerifyObjectState(),
                               transitions={'RV_h': 'GripObject', 'RV_v': 'GripObject',
                                            'black_gray': 'GripObject',
                                            'cannot_place': 'not_implemented',
                                            'success': 'GripObject',
                                            'failed': 'should_not_happen'},
                               remapping={'pose_in': 'pose', 'point_out': 'point',
                                          'object_out': 'object', 'backplate_pose_out': 'backplate_pose'})
        smach.StateMachine.add('GripObject', Grip(),
                               transitions={'success': 'PreGrip_after_Grip', 'too_far': 'should_not_happen',
                                            'failed': 'RecoverArmToPreGrip',
                                            'failed_after_grip': 'RecoverArmToPreGrip'},
                               remapping={'pose_in': 'pose', 'point_in': 'point', 'object_in': 'object',
                                          'object_out': 'object'})
        smach.StateMachine.add('RecoverArmToPreGrip', recover_arm_to_pre_grip(),
                               transitions={'success': 'PreGrip', 'failed': 'arm_failed'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('PreGrip_after_Grip', PreGrip(blocking=True),
                               transitions={'success': 'AddToBackplate'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('AddToBackplate',AddToBackplate(),
                               transitions={'success': 'done'},
                               remapping={'object_in': 'object'})
        return sm


def pick_from_platform_smach():
    sm = smach.StateMachine(outcomes=['done', 'arm_failed', 'not_implemented', 'should_not_happen'],
                            input_keys=['pose'])
    with sm:
        smach.StateMachine.add('PreGrip', PreGrip(blocking=True), transitions={'success': 'AlignToPlatformState'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('AlignToPlatformState', AlignToPlatformState(),
                               transitions={'reached': 'SwitchVisionOn',
                                            'failed': 'TuckForDrive'},
                               remapping={'pose_in': 'pose'})
        # smach.StateMachine.add('MoveTowardsPlatform', MoveTowardsPlatform(0.05, 0.075),
        # transitions={'done': 'AlignToPlatformState'},
        #                        remapping={'pose_in': 'pose'})
        smach.StateMachine.add('SwitchVisionOn', SwitchObjectHoleManagerOnState(),
                               transitions={'done': 'MoveBack'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('MoveBack', MoveBackWithAlign(0.5, 0.075),
                               transitions={'done': 'SwitchVisionOff'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('SwitchVisionOff', SwitchObjectHoleManagerOffState(),
                               transitions={'done': 'GetNextObject'})

        # while until nothing left
        smach.StateMachine.add('GetNextObject', GetNextObjectLocationState(),
                               transitions={'success': 'AlignToObject', 'nothing_found': 'TuckForDrive',
                                            'nothing_left': 'TuckForDrive', 'backplate_full': 'TuckForDrive'},
                               remapping={'pose_in': 'pose', 'point_out': 'point'})
        smach.StateMachine.add('AlignToObject', AlignToObjectState(eps_lin=0.01, eps_theta=0.02, counter=5),
                               transitions={'reached': 'VerifyObject', 'failed': 'VerifyObject'},
                               remapping={'point_in': 'point', 'pose_in': 'pose'})
        smach.StateMachine.add('VerifyObject', VerifyObjectState(),
                               transitions={'RV_h': 'RV20_h_check', 'RV_v': 'not_implemented',
                                            'black_gray': 'not_implemented',
                                            'cannot_place': 'BlacklistObject',
                                            'success': 'GripObject',
                                            'failed': 'BlacklistObject'},
                               remapping={'pose_in': 'pose', 'point_out': 'point',
                                          'object_out': 'object', 'backplate_pose_out': 'backplate_pose'})
        smach.StateMachine.add('BlacklistObject', BlacklistObject(),
                               transitions={'done': 'GetNextObject'},
                               remapping={'point_in': 'point', 'pose_in': 'pose'})
        smach.StateMachine.add('RV20_h_check', RV20_h_check(),
                               transitions={'correct': 'PlaceOnPlatform', 'incorrect': 'PreGrip_after_POP',
                                            'arm_failed': 'arm_failed', 'should_not_happen': 'should_not_happen'},
                               remapping={'pose': 'pose', 'point': 'point', 'object': 'object',
                                          'object_out': 'object',
                                          'backplate_pose_out': 'backplate_pose'})

        smach.StateMachine.add('GripObject', Grip(),
                               transitions={'success': 'PlaceOnPlatform', 'too_far': 'BlacklistObject',
                                            'failed': 'RecoverArmToPreGrip',
                                            'failed_after_grip': 'RecoverArmToPreGrip'},
                               remapping={'pose_in': 'pose', 'point_in': 'point', 'object_in': 'object',
                                          'object_out': 'object'})
        smach.StateMachine.add('RecoverArmToPreGrip', recover_arm_to_pre_grip(),
                               transitions={'success': 'BlacklistObject', 'failed': 'arm_failed'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('PlaceOnPlatform', PlaceOnPlatform(),
                               transitions={'success': 'DeleteObjectFromLocation'},
                               remapping={'object_in': 'object', 'backplate_pose_in': 'backplate_pose'})
        smach.StateMachine.add('DeleteObjectFromLocation', DeleteObjFromLoc(),
                               transitions={'continue': 'RotateToPlatform', 'nothing_left': 'TuckForDrive'},
                               remapping={'object_in': 'object', 'pose_in': 'pose', 'point_in': 'point'})

        smach.StateMachine.add('RotateToPlatform', RotateToPlatform(),
                               transitions={'done': 'PreGrip_after_POP'},
                               remapping={'pose_in': 'pose'})

        smach.StateMachine.add('PreGrip_after_POP', PreGrip(blocking=True),
                               transitions={'success': 'ShouldAlign'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('ShouldAlign', ShouldAlign(min_distance=0.02),
                               transitions={'align': 'ReAlign', 'continue': 'GetNextObject'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('ReAlign', AlignToPlatformState(),
                               transitions={'reached': 'GetNextObject',
                                            'failed': 'GetNextObject'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('TuckForDrive', TuckForDrive(blocking=True),
                               transitions={'done': 'done'},
                               remapping={'pose_in': 'pose'})

    return sm


def RV20_h_check():
    sm = smach.StateMachine(outcomes=['correct', 'incorrect', 'arm_failed', 'should_not_happen'],
                            input_keys=['point', 'object', 'pose'], output_keys=['object_out', 'backplate_pose_out'])

    with sm:
        smach.StateMachine.add('GripObject', Grip(),
                               transitions={'success': 'RV20CheckArmPosition', 'too_far': 'BlacklistObject',
                                            'failed': 'RecoverArmToPreGrip',
                                            'failed_after_grip': 'RecoverArmToPreGrip'},
                               remapping={'pose_in': 'pose', 'point_in': 'point', 'object_in': 'object',
                                          'object_out': 'object_out'})
        smach.StateMachine.add('RV20CheckArmPosition', RV20CheckArmPosition(),
                               transitions={'done': 'VerifyRV20Vision'})
        smach.StateMachine.add('VerifyRV20Vision', RV20CheckVision(),
                               transitions={'correct': 'correct', 'incorrect': 'PlaceBackOnLocation'},
                               remapping={'pose_in': 'pose', 'object_in': 'object', 'object_out': 'object_out',
                                          'backplate_pose_out': 'backplate_pose_out'})
        smach.StateMachine.add('PlaceBackOnLocation', Place(),
                               transitions={'too_far': 'should_not_happen', 'success': 'BlacklistObject',
                                            'failed': 'RecoverArmToPreGrip', 'failed_after_place': 'BlacklistObject'},
                               remapping={'pose_in': 'pose', 'point_in': 'point', 'object_in': 'object'})

        smach.StateMachine.add('RecoverArmToPreGrip', recover_arm_to_pre_grip(),
                               transitions={'success': 'BlacklistObject', 'failed': 'arm_failed'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('BlacklistObject', BlacklistObject(),
                               transitions={'done': 'incorrect'},
                               remapping={'point_in': 'point', 'pose_in': 'pose'})
    return sm

def ppt_place():
    sm = smach.StateMachine(outcomes=['done', 'arm_failed', 'should_not_happen'], input_keys=['pose'])
    with sm:
        smach.StateMachine.add('PreGrip_PPT', PreGrip(blocking=True),
                               transitions={'success': 'AlignToPlatformState_PPT'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('AlignToPlatformState_PPT', AlignToPlatformState(),
                               transitions={'reached': 'SwitchVisionOn_PPT',
                                            'failed': 'TuckForDrive'},
                               remapping={'pose_in': 'pose'})
        # smach.StateMachine.add('MoveTowardsPlatform', MoveTowardsPlatform(0.05, 0.075),
        # transitions={'done': 'AlignToPlatformState_PPT'},
        #                        remapping={'pose_in': 'pose'})
        smach.StateMachine.add('SwitchVisionOn_PPT', SwitchObjectHoleManagerOnState(holes=True),
                               transitions={'done': 'MoveBack_PPT'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('MoveBack_PPT', MoveBackWithAlign(0.5, 0.075), transitions={'done': 'SwitchVisionOff_PPT'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('SwitchVisionOff_PPT', SwitchObjectHoleManagerOffState(holes=True),
                               transitions={'done': 'GetNextHole'})
        # # while until nothing left
        # why nothing_found toDo
        smach.StateMachine.add('GetNextHole', GetNextObjectLocationState(holes=True),
                               transitions={'success': 'AlignToHole', 'nothing_found': 'TuckForDrive',
                                            'nothing_left': 'TuckForDrive', 'backplate_full': 'should_not_happen'},
                               remapping={'pose_in': 'pose', 'point_out': 'point'})
        smach.StateMachine.add('AlignToHole', AlignToObjectState(eps_lin=0.01, eps_theta=0.02, counter=5),
                               transitions={'reached': 'VerifyHole', 'failed': 'VerifyHole'},
                               remapping={'point_in': 'point', 'pose_in': 'pose'})
        smach.StateMachine.add('VerifyHole', VerifyHoleState(),
                               transitions={'success': 'PickFromPlatform_PPT', 'too_far': 'BlacklistHole',
                                            'failed': 'BlacklistHole'},
                               remapping={'pose_in': 'pose', 'point_out': 'point',
                                          'object_out': 'object', 'backplate_pose_out': 'backplate_pose'})
        smach.StateMachine.add('BlacklistHole', BlacklistHole(),
                               transitions={'done': 'GetNextHole'},
                               remapping={'point_in': 'point', 'pose_in': 'pose'})
        smach.StateMachine.add('PickFromPlatform_PPT', PickFromPlatform(),
                               transitions={'success': 'PlaceFromPlatform_PPT'},
                               remapping={'object_in': 'object', 'backplate_pose_in': 'backplate_pose'})
        smach.StateMachine.add('PlaceFromPlatform_PPT', Place(),
                               transitions={'too_far': 'should_not_happen', 'success': 'PreGrip_after_place_PPT',
                                            'failed': 'RecoverArmToPreGrip',
                                            'failed_after_place': 'RecoverArmToPreGrip'},
                               remapping={'pose_in': 'pose', 'point_in': 'point', 'object_in': 'object'})
        smach.StateMachine.add('PreGrip_after_place_PPT', PreGrip(blocking=True),
                               transitions={'success': 'ShouldAlign_PPT'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('ShouldAlign_PPT', ShouldAlign(min_distance=0.02),
                               transitions={'align': 'ReAlign_PPT', 'continue': 'GetNextHole'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('ReAlign_PPT', AlignToPlatformState(),
                               transitions={'reached': 'GetNextHole',
                                            'failed': 'GetNextHole'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('TuckForDrive', TuckForDrive(blocking=True),
                               transitions={'done': 'done'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('RecoverArmToPreGrip', recover_arm_to_pre_grip(),
                               transitions={'success': 'BlacklistHole', 'failed': 'arm_failed'},
                               remapping={'pose_in': 'pose'})

    return sm


def place_at_location():
    sm = smach.StateMachine(outcomes=['done', 'should_not_happen', 'arm_failed'], input_keys=['pose'])
    with sm:
        smach.StateMachine.add('PreGripPlace', PreGrip(), transitions={'success': 'AlignToPlacePlatformState'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('AlignToPlacePlatformState', AlignToPlatformState(),
                               transitions={'reached': 'DestinationPlaceManager',
                                            'failed': 'TuckForDrive'},
                               remapping={'pose_in': 'pose'})
        # smach.StateMachine.add('MoveTowardsPlatform', MoveTowardsPlatform(0.05, 0.075),
        # transitions={'done': 'AlignToPlacePlatformState'},
        #                        remapping={'pose_in': 'pose'})
        smach.StateMachine.add('DestinationPlaceManager', DestinationPlaceManager(),
                               transitions={'place': 'PickFromPlatform', 'nothing_left': 'ArmUp'},
                               remapping={'pose_in': 'pose', 'point_out': 'point', 'object_out': 'object',
                                          'backplate_pose_out': 'backplate_pose'})
        smach.StateMachine.add('PickFromPlatform', PickFromPlatform(),
                               transitions={'success': 'PlaceFromPlatform'},
                               remapping={'object_in': 'object', 'backplate_pose_in': 'backplate_pose'})
        smach.StateMachine.add('PlaceFromPlatform', Place(),
                               transitions={'too_far': 'should_not_happen', 'success': 'DestinationPlaceManager',
                                            'failed': 'RecoverArmToTuck', 'failed_after_place': 'RecoverArmToTuck'},
                               remapping={'pose_in': 'pose', 'point_in': 'point', 'object_in': 'object'})
        smach.StateMachine.add('RecoverArmToTuck', recover_arm_to_tuck(),
                               transitions={'success': 'DestinationPlaceManager', 'failed': 'arm_failed'})
        smach.StateMachine.add('ArmUp', ArmUp(blocking=True), transitions={'done': 'TuckForDrive'})
        smach.StateMachine.add('TuckForDrive', TuckForDrive(blocking=False),
                               transitions={'done': 'LeaveLocation'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('LeaveLocation', LeaveLocationState(),
                               transitions={'done': 'done'})
    return sm


def recover_arm_to_tuck():
    sm = smach.StateMachine(outcomes=['success', 'failed'])
    with sm:
        smach.StateMachine.add('OpenGripper', OpenGripper(), transitions={'done': 'TuckArm'})
        smach.StateMachine.add('TuckArm', TuckArm(),
                               transitions={'success': 'TuckForDrive', 'failed': 'failed', 'not_reached': 'TuckArm'})
        smach.StateMachine.add('TuckForDrive', TuckForDrive(blocking=True, no_rotate=True),
                               transitions={'done': 'success'})
    return sm


def recover_arm_to_pre_grip():
    sm = smach.StateMachine(outcomes=['success', 'failed'], input_keys=['pose_in'])
    with sm:
        smach.StateMachine.add('OpenGripper', OpenGripper(), transitions={'done': 'TuckArm'})
        smach.StateMachine.add('TuckArm', TuckArm(),
                               transitions={'success': 'PreGrip', 'failed': 'failed', 'not_reached': 'TuckArm'})
        smach.StateMachine.add('PreGrip', PreGrip(blocking=True), transitions={'success': 'success'},
                               remapping={'pose_in': 'pose_in'})
    return sm


def bnt():
    sm = smach.StateMachine(outcomes=['end'])
    with sm:
        # #####BNT SPECIFIC
        smach.StateMachine.add('TuckForDrive_BNT', TuckForDrive(blocking=True, no_rotate=True),
                               transitions={'done': 'GetNextGoal'})

        smach.StateMachine.add('GetNextGoal', GetNextBNTLoc(), transitions={'done': 'MoveToNext_BNT'},
                               remapping={'pose_out': 'pose'})
        # ## MOVE STATE WITH RECOVER
        smach.StateMachine.add('MoveToNext_BNT', MoveStateUserData(),
                               transitions={'reached': 'DecideAfterMove_BNT', 'not_reached': 'RecoverMove_BNT',
                                            'failed': 'end'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('RecoverMove_BNT', RecoverState(), transitions={'done': 'MoveToNext_BNT'})
        # ## END MOVE STATE WITH RECOVER

        # #Decision state after Move:  ## Only BNT possible
        smach.StateMachine.add('DecideAfterMove_BNT', DecideAfterMoveState(),
                               transitions={'BNT': 'ScanMatcher_BNT', 'Pickup': 'end',
                                            'Place': 'end', 'End': 'end'},
                               remapping={'pose_in': 'pose'})

        smach.StateMachine.add('ScanMatcher_BNT', ScanMatcher(),
                               transitions={'reached': 'SleepState', 'not_reached': 'ScanMatcher_BNT',
                                            'failed': 'SleepState'},
                               remapping={'pose_in': 'pose'})
        smach.StateMachine.add('SleepState', SleepState(), transitions={'done': 'DeleteCurGoal'},
                               remapping={'pose_in': 'pose'})
        # DELETE CURRENT GOAL AND GET NEXT GOAL
        smach.StateMachine.add('DeleteCurGoal', DeleteCurrentGoalState(), transitions={'done': 'GetNextGoal'})

        # #######END BNT
    return sm


def set_params(sm):
    sm.userdata.pose = rospy.get_param('locations')[0]


def add_objects(sm):
    add_to_platform = rospy.ServiceProxy('/backplate_manager/add_object_to_plate', BackplateStateModify)
    obj_msg = BackplateObject()
    obj_msg.label = 'F20_20_B_h'
    obj_msg.obj_name = 'F20_20_B'
    obj_msg.place_type = 'PPT'
    obj_msg.destination_loc = 'left'
    obj_msg.place_loc = 'center_2'
    # obj_msg.rotation = obj['rotation']
    add_to_platform(obj_msg)
    obj_msg.label = 'M30_h'
    obj_msg.obj_name = 'M30'
    obj_msg.place_loc = 'center_3'
    add_to_platform(obj_msg)
    obj_msg.label = 'S40_40_G_h'
    obj_msg.obj_name = 'S40_40_G'
    obj_msg.place_loc = 'center_1'
    add_to_platform(obj_msg)
    # add_to_platform(obj_msg)


def reset():
    clear_backplate = rospy.ServiceProxy('/backplate_manager/remove_all_objects_from_plate', Empty)
    clear_destinations = rospy.ServiceProxy('/smach/reset_destination_manager', Empty)
    reset_hole_manager = rospy.ServiceProxy('/vision/hole_manager/reset', Empty)
    reset_object_manager = rospy.ServiceProxy('/vision/object_manager/reset', Empty)
    try:
        clear_backplate()
    except Exception as e:
        print "clear backplate failed", e
    try:
        clear_destinations()
    except Exception as e:
        print "clear destinations failed", e
    try:
        reset_hole_manager()
    except Exception as e:
        print "Reset_hole manager failed", e
    try:
        reset_object_manager()
    except Exception as e:
        print "reset object manager failed", e


class SmachShow():
    def __init__(self):
        self.sm = smach.StateMachine(outcomes=['end'])

        self.ppt_smach = ppt_place_from_gripper()
        self.pick = pick_and_keep()
        with self.sm:
            smach.StateMachine.add('SetToPick', SetPose("left", "normal"),
                                   transitions={'success': 'Pick'},
                                   remapping={'pose_out':'pose'})
            smach.StateMachine.add('Pick', self.pick,
                                   transitions={'done': 'SetToPPT', 'arm_failed': 'end', 'not_implemented': 'end',
                                                'should_not_happen': 'end'})
            smach.StateMachine.add('SetToPPT', SetPose("front", "PPT"),
                                   transitions={'success': 'PPT'},
                                   remapping={'pose_out':'pose'})
            smach.StateMachine.add('PPT', self.ppt_smach,
                                   transitions={'done': 'end', 'arm_failed': 'end', 'should_not_happen': 'end'})
                # Create and start the introspection server
        self.sis = smach_ros.IntrospectionServer('server_name', self.sm, '/SLAW_SMACH')
        self.sis.start()
        self.serv = rospy.Service("/start_SMACH", Empty, self.go)
        
    def go(self, req):
        reset()
        print "Starting SMACH"
        publish_status("I got the specifications <br> and will start now.")
        self.sm.execute()
        publish_status("I am finished.")

        return EmptyResponse()

    def stop(self):
        self.sis.stop()

if __name__ == '__main__':
    rospy.init_node('smach_test')

    # reset all states and managers
    reset()

    smach = SmachShow()
    rospy.spin()
    # #create StateMachines

    # sm = pick_from_platform_smach()
    # sm = place_at_location()
    # sm = ppt_place()

    # set_params(sm)
    # add_objects(sm)
    # Create and start the introspection server


    # sis = smach_ros.IntrospectionServer('smach_test', sm, '/SM_TEST')
    #sis.start()

    # Execute SMACH plan
    #outcome = sm.execute()

    # stop server
    #sis.stop()
