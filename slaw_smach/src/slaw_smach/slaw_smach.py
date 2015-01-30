#!/usr/bin/env python
import rospy

from ArmStates import *
from MoveStates import *
from ObjectDetectState import *
from DecisionStates import *

from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Bool
## TODO after Eindhoven: Add failsafe if hole not detected
## add states if object too far or too close to gripper

class Smach():
    def __init__(self):
        rospy.init_node('slaw_smach')
        self.sm = smach.StateMachine(outcomes=['end'])
        

        
        with self.sm:

            ### MOVE STATE WITH RECOVER
            smach.StateMachine.add('MoveToNext', MoveStateUserData(), transitions = {'reached':'DecideAfterMove', 'not_reached': 'RecoverMove', 'failed': 'DeleteCurGoal'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})
            
            smach.StateMachine.add('RecoverMove', RecoverState(), transitions = {'done':'MoveToNext'}, remapping = {'pose_in':'pose', 'pose_out': 'pose'})
            ### END MOVE STATE WITH RECOVER

            ##Decision state after Move:
            smach.StateMachine.add('DecideAfterMove', DecideAfterMoveState(),transitions = {'BNT': 'ScanMatcher_BNT', 'Pickup':'ScanMatcher_Pickup', 'Place':'ScanMatcher_Place', 'End':'end'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})
            
            ######BNT SPECIFIC

            smach.StateMachine.add('ScanMatcher_BNT', ScanMatcher(), transitions = {'reached':'SleepState', 'not_reached':'ScanMatcher_BNT', 'failed':'SleepState'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})

            smach.StateMachine.add('SleepState', SleepState(), transitions = {'done':'DeleteCurGoal'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})
        
            ########END BNT

            ##### DELETE CURRENT GOAL OR GET NEXT GOAL
            smach.StateMachine.add('DeleteCurGoal', DeleteCurrentGoalState(), transitions = {'done':'MoveToNext'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})
            
            smach.StateMachine.add('GetNextGoal', GetNextGoalState(), transitions = {'done':'MoveToNext'}, remapping = {'pose_in':'pose','object_in':'object', 'pose_out':'pose'})
            ##### END DELETE CURRENT GOAL OR GET NEXT GOAL

            

            ### PICKUP
            smach.StateMachine.add('ScanMatcher_Pickup', ScanMatcher(), transitions = {'reached':'DecideBeforePreGrip', 'not_reached':'ScanMatcher_Pickup', 'failed':'MoveToNext'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})
            #smach.StateMachine.add('ScanMatcher_Pickup', ScanMatcher(), transitions = {'reached':'ScanMatcher_Align', 'not_reached':'ScanMatcher_Pickup', 'failed':'MoveToNext'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})
            #smach.StateMachine.add('ScanMatcher_Align', AlignState(), transitions = {'done':'DecideBeforePreGrip'})
            ##
            
            #Either CBT Pickup or normal Pickup
            smach.StateMachine.add('DecideBeforePreGrip', DecideBeforePreGripState(),transitions = {'CBT': 'PreGrip_CBT', 'Pickup':'PreGrip'}, remapping = {'pose_in':'pose', 'pose_out':'pose', 'dist_out':'dist'})
            
            ######CBT STUFF
            smach.StateMachine.add('PreGrip_CBT', PreGripCBT(), transitions = {'success':'ScanForObjectCBT', 'failed':'TuckArmPreGripCBT'},remapping = {'pose_in':'pose', 'pose_out':'pose'})
            smach.StateMachine.add('TuckArmPreGripCBT', TuckArm(), transitions = {'success':'PreGrip_CBT', 'not_reached':'TuckArmPreGripCBT','failed':'end'})
            smach.StateMachine.add('ScanForObjectCBT', ScanForObjectCBT(), transitions = {'success':'GripCBT'})
            smach.StateMachine.add('GripCBT', GripCBT(), transitions = {'end':'DeleteCurGoal'})
            #### END CBT Stuff


            ### NORMAL PICKUP
            smach.StateMachine.add('PreGrip', PreGrip(), transitions = {'success':'Scan', 'failed':'TuckArmPreGrip'},remapping = {'pose_in':'pose', 'pose_out':'pose'})
            smach.StateMachine.add('TuckArmPreGrip', TuckArm(), transitions = {'success':'PreGrip', 'not_reached':'TuckArmPreGrip','failed':'end'})

            #scan
            smach.StateMachine.add("Scan", ScanForObjectsState(), transitions = {'success': 'Grip', 'failed':'TuckArmMoveNext','nothing_found': 'TuckArmDelete'}, remapping = {'pose_in':'pose', 'pose_out':'pose', 'object_out':'object', 'point_out':'point', 'dist_in':'dist','dist_out':'dist'})

            #if misdetection try again 
            smach.StateMachine.add('TuckArmMoveNext', TuckArm(), transitions = {'success':'MoveToNext', 'not_reached':'TuckArmMoveNext','failed':'end'})

            #if nothing found try next Goal
            smach.StateMachine.add('TuckArmDelete', TuckArm(), transitions = {'success':'DeleteCurGoal', 'not_reached':'TuckArmDelete','failed':'end'})

            #Grip Object
            smach.StateMachine.add("Grip", Grip(), transitions = {'success':'DecideRV20', 'too_far':'ScanMatcher_Pickup', 'failed':'TuckArmFailGrip', 'failed_after_grip':'TuckArmGrip'}, remapping = {'pose_in':'pose', 'object_in':'object', 'point_in':'point','pose_out':'pose', 'object_out':'object', 'point_out':'point'})
            
            #Decide RV20:
            smach.StateMachine.add('DecideRV20', DecideRV20State(),transitions = {'RV20': 'TuckForDriveAfterGrip', 'Normal':'TuckForDriveAfterGrip'}, remapping = {'object_in':'object', 'object_out':'object'})
            #smach.StateMachine.add('DecideRV20', DecideRV20State(),transitions = {'RV20': 'RV20CheckArm', 'Normal':'TuckForDriveAfterGrip'}, remapping = {'object_in':'object', 'object_out':'object', 'pose_out':'pose'})

            
            ####CHECK if RV20 which one
            smach.StateMachine.add('RV20CheckArm', RV20CheckState(), transitions = {'success':'RV20CheckVision','failed':'TuckArmPreCheckArm'}, remapping = {'pose_in':'pose'})
            smach.StateMachine.add('TuckArmPreCheckArm', TuckArm(), transitions = {'success':'RV20CheckArm', 'not_reached':'TuckArmPreCheckArm','failed':'end'})

            #smach.StateMachine.add('RV20CheckVision', RV20CheckVision(), transitions = {'success':'RV20RotateTake','failed':'RV20RotateReplace'}, remapping = {'pose_in':'pose', 'object_in':'object', 'pose_out':'pose'})
            smach.StateMachine.add('RV20CheckVision', RV20CheckVision(), transitions = {'success':'RV20RotateTake','failed':'RV20Trash'}, remapping = {'pose_in':'pose', 'object_in':'object', 'pose_out':'pose'})
            smach.StateMachine.add('RV20Trash', RV20Trash(), transitions = {'done':'PreGrip'})
            #smach.StateMachine.add('RV20RotateReplace', RV20ReplaceObjectRotate(), transitions = {'success':'RV20Replace','failed':'RV20Replace'}, remapping = {'pose_in':'pose'})
            smach.StateMachine.add('RV20RotateTake', RV20ReplaceObjectRotate(), transitions = {'success':'TuckForDriveAfterGrip','failed':'TuckForDriveAfterGrip'}, remapping = {'pose_in':'pose'})
                                   
            
            #smach.StateMachine.add('RV20Replace', FinePlace(), transitions = {'success':'RV20ReplaceUp', 'failed':'TuckArmFailPlace_RV20', 'too_far':'RV20Replace','failed_after_place':'TuckArmFailPlace_RV20'}, remapping = {'object_in':'object','pose_in':'pose', 'pose_out':'pose', 'point_in':'point'})
            #smach.StateMachine.add('TuckArmFailPlace_RV20', TuckArm(), transitions = {'success':'RV20Replace', 'not_reached':'TuckArmFailPlace_RV20','failed':'end'})

            #smach.StateMachine.add('RV20ReplaceUp', RV20ReplaceUp(), transitions = {'done':'MoveBack10'})

            #MoveBack 10 to skip object and resume scanning
            #smach.StateMachine.add('MoveBack10', MoveBack(0.10), transitions = {'done':'Remove10'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})
            #smach.StateMachine.add('Remove10', RemoveDist(0.10), transitions = {'done':'PreGrip'}, remapping = {'dist_in':'dist', 'dist_out':'dist'})

            #Tuck and Move away
            ##Tuck For Drive
            smach.StateMachine.add('TuckForDriveAfterGrip', TuckForDrive(), transitions={'done':'MoveAwayFromPlatform'}, remapping = {'pose_in':'pose'} )

            
            smach.StateMachine.add('TuckArmGrip', TuckArm(), transitions = {'success':'MoveAwayFromPlatform', 'not_reached':'TuckArmGrip','failed':'end'})
            smach.StateMachine.add('TuckArmFailGrip', TuckArm(), transitions = {'success':'MoveToNext', 'not_reached':'TuckArmFailGrip','failed':'end'})
            smach.StateMachine.add('MoveAwayFromPlatform', RecoverState(), transitions = {'done':'MoveToPlace'})

            ### Move to Place location
            smach.StateMachine.add('MoveToPlace', MoveStateUserData(), transitions = {'reached': 'ScanMatcher_Place', 'not_reached': 'MoveAwayFromPlatform', 'failed': 'MoveAwayFromPlatform'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})

            smach.StateMachine.add('ScanMatcher_Place', ScanMatcher(), transitions = {'reached':'DecideBeforePlace', 'not_reached':'ScanMatcher_Place', 'failed':'DecideBeforePlace'}, remapping = {'pose_in':'pose', 'suffix_in':'suffix', 'pose_out':'pose'})

            #### Decide either Normal place or PPT place
            smach.StateMachine.add('DecideBeforePlace', DecideBeforePlaceState(),transitions = {'PPT': 'PreScanHole', 'Normal':'MoveBack'}, remapping = {'object_in':'object', 'object_out':'object'})

            ####PPT
            smach.StateMachine.add('PreScanHole', PreGrip(), transitions = {'success':'ScanHole', 'failed':'TuckArmPreScan'},remapping = {'pose_in':'pose', 'pose_out':'pose'})

            smach.StateMachine.add('TuckArmPreScan', TuckArm(), transitions = {'success':'PreScanHole', 'not_reached':'TuckArmPreScan','failed':'end'})

            smach.StateMachine.add("ScanHole", ScanForHoles(), transitions = {'success': 'FinePlace', 'failed':'ScanMatcher_Place','nothing_found': 'ScanMatcher_Place'}, remapping = {'pose_in':'pose', 'pose_out':'pose', 'object_in':'object', 'object_out':'object', 'point_out':'point'})

                               
            smach.StateMachine.add('FinePlace', FinePlace(), transitions = {'success':'TuckForDriveAfterPlace', 'failed':'TuckArmFailPlace_PPT', 'too_far':'ScanMatcher_Place','failed_after_place':'TuckArmFailPlace_PPT'}, remapping = {'object_in':'object','pose_in':'pose', 'pose_out':'pose', 'point_in':'point'})

            smach.StateMachine.add('TuckArmFailPlace_PPT', TuckArm(), transitions = {'success':'FinePlace', 'not_reached':'TuckArmFailPlace_PPT','failed':'end'})
            ### END PPT


            ##NORMAL PLACE
            smach.StateMachine.add('MoveBack', MoveBack(0.25), transitions = {'done':'Place'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})
            smach.StateMachine.add('Place', Place(), transitions = {'success':'TuckForDriveAfterPlace', 'failed':'TuckArmFailPlace'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})

            ##Tuck For Drive
            smach.StateMachine.add('TuckForDriveAfterPlace', TuckForDrive(), transitions={'done':'MoveAwayFromPlatformAfterPlace'}, remapping = {'pose_in':'pose'} )

            smach.StateMachine.add('TuckArmFailPlace', TuckArm(), transitions = {'success':'Place', 'not_reached':'TuckArmFailPlace','failed':'end'})

            smach.StateMachine.add('MoveAwayFromPlatformAfterPlace', RecoverState(), transitions = {'done':'GetNextGoal'})


        # Create and start the introspection server
        self.sis = smach_ros.IntrospectionServer('server_name', self.sm, '/SLAW_SMACH')
        self.sis.start()
        self.serv = rospy.Service("/start_SMACH", Empty, self.go)

        
    def go(self, req):
        #sm.userdata.pose = "D2"

        print "Starting SMACH"
        locations = rospy.get_param('locations')
        self.sm.userdata.pose = locations[0]
        #self.sm.userdata.suffix = "_grip"
        self.sm.execute()
        return EmptyResponse()

    def stop(self):
        self.sis.stop()
        
if __name__ == '__main__':
    smach = Smach()
    rospy.spin()
    smach.stop()
  
