#!/usr/bin/env python
import rospy
import smach_ros
from ArmStates import *
from MoveStates import *
from ObjectDetectState import *
from DecisionStates import *
from smach_helpers import publish_status
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
from std_msgs.msg import Bool
from subtask_smachs import ppt_place, place_at_location, pick_from_platform_smach, bnt, reset
from ChooseTargetPosition import ChooseTargetPositionManipulation
# # TODO after Eindhoven: Add failsafe if hole not detected
# # add states if object too far or too close to gripper


class Smach():
    def __init__(self):
        rospy.init_node('slaw_smach')
        self.sm = smach.StateMachine(outcomes=['end'])

        self.ppt_smach = ppt_place()
        self.place_at_loc = place_at_location()
        self.pick_from_loc = pick_from_platform_smach()
        self.bnt = bnt()
        with self.sm:
            smach.StateMachine.add('DecideTest', DecideTest(),
                                   transitions={'BNT': 'BNT_SMACH',
                                                'Manipulation': 'TuckForDrive',
                                                'NoPlan': 'end'})
            smach.StateMachine.add('TuckForDrive', TuckForDrive(blocking=True, no_rotate=True),
                                   transitions={'done': 'ChooseTargetPositionManipulation'})

            smach.StateMachine.add('ChooseTargetPositionManipulation', ChooseTargetPositionManipulation(),
                                   transitions={'done': 'MoveToNext'},
                                   remapping={'pose_out': 'pose'})

            # ## MOVE STATE WITH RECOVER
            smach.StateMachine.add('MoveToNext', MoveStateUserData(),
                                   transitions={'reached': 'DecideAfterMove', 'not_reached': 'RecoverMove',
                                                'failed': 'end'},
                                   remapping={'pose_in': 'pose'})
            smach.StateMachine.add('RecoverMove', RecoverState(), transitions={'done': 'MoveToNext'})
            # ## END MOVE STATE WITH RECOVER

            # #Decision state after Move: ##BNT is not possible
            smach.StateMachine.add('DecideAfterMove', DecideAfterMoveState(),
                                   transitions={'BNT': 'end', 'Pickup': 'ScanMatcher_Pickup',
                                                'Place': 'ScanMatcher_Place', 'End': 'end'},
                                   remapping={'pose_in': 'pose'})

            # #####BNT
            smach.StateMachine.add('BNT_SMACH', self.bnt, transitions={'end': 'end'})
            # #######END BNT

            # ## PICKUP
            smach.StateMachine.add('ScanMatcher_Pickup', ScanMatcher(),
                                   transitions={'reached': 'DecideBeforePreGrip', 'not_reached': 'ScanMatcher_Pickup',
                                                'failed': 'ChooseTargetPositionManipulation'},
                                   remapping={'pose_in': 'pose'})

            # # TODO CBT has to come here!!!
            # Either CBT Pickup or normal Pickup
            smach.StateMachine.add('DecideBeforePreGrip', DecideBeforePreGripState(),
                                   transitions={'CBT': 'end', 'Pickup': 'PickFromLoc'},
                                   remapping={'pose_in': 'pose'})

            smach.StateMachine.add('PickFromLoc', self.pick_from_loc,
                                   transitions={'done': 'MoveAwayFromPlatform', 'arm_failed': 'end',
                                                'not_implemented': 'end', 'should_not_happen': 'end'},
                                   remapping={'pose': 'pose'})

            smach.StateMachine.add('MoveAwayFromPlatform', RecoverState(),
                                   transitions={'done': 'ChooseTargetPositionManipulation'})


            # ## Placing
            smach.StateMachine.add('ScanMatcher_Place', ScanMatcher(),
                                   transitions={'reached': 'DecideBeforePlace', 'not_reached': 'ScanMatcher_Place',
                                                'failed': 'DecideBeforePlace'},
                                   remapping={'pose_in': 'pose'})

            # ### Decide either Normal place or PPT place
            smach.StateMachine.add('DecideBeforePlace', DecideBeforePlaceState(),
                                   transitions={'PPT': 'PPT_Smach', 'Normal': 'PlaceAtLoc', 'nothing_found': 'end'},
                                   remapping={'pose_in': 'pose'})

            ####PPT
            smach.StateMachine.add('PPT_Smach', self.ppt_smach,
                                   transitions={'done': 'MoveAwayFromPlatform', 'arm_failed': 'end',
                                                'should_not_happen': 'end'},
                                   remapping={'pose': 'pose'})
            ### END PPT


            ##NORMAL PLACE
            smach.StateMachine.add('PlaceAtLoc', self.place_at_loc,
                                   transitions={'done': 'ChooseTargetPositionManipulation', 'should_not_happen': 'end',
                                                'arm_failed': 'end'},
                                   remapping={'pose': 'pose'})


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
    smach = Smach()
    # smach.go(EmptyRequest())
    rospy.spin()
    smach.stop()
