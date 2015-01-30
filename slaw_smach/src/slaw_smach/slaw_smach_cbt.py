#!/usr/bin/env python
import rospy
import smach

from ArmStates import *
from MoveStates import *
from ObjectDetectState import *

from std_srvs.srv import Empty, EmptyResponse


class SmachCBT:
    def __init__(self):
        rospy.init_node('cbt_smach_test')
        self.sm = smach.StateMachine(outcomes=['end'])
        
        
        with self.sm:
            smach.StateMachine.add('MoveToStart', MoveStateUserData(), transitions = {'reached': 'ScanMatcher', 'not_reached': 'RecoverToStart', 'failed': 'MoveToStart'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})
            
            smach.StateMachine.add('RecoverToStart', RecoverState(), transitions = {'done':'MoveToStart'}, remapping = {'pose_in':'pose', 'pose_out': 'pose'})

        
            smach.StateMachine.add('ScanMatcher', ScanMatcher(), transitions = {'reached':'PreGrip', 'not_reached':'ScanMatcher', 'failed':'PreGrip'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})
            
            smach.StateMachine.add('PreGrip', PreGripCBT(), transitions = {'success':'ScanForObject', 'failed':'TuckArmPreGrip'},remapping = {'pose_in':'pose', 'pose_out':'pose'})

            smach.StateMachine.add('TuckArmPreGrip', TuckArm(), transitions = {'success':'PreGrip', 'not_reached':'TuckArmPreGrip','failed':'end'})

            smach.StateMachine.add('ScanForObject', ScanForObjectCBT(), transitions = {'success':'TuckArmEnd'})
            smach.StateMachine.add('TuckArmEnd', TuckArmCBT(), transitions = {'success':'end', 'not_reached':'end','failed':'end'})
        
       

        # Create and start the introspection server
        self.sis = smach_ros.IntrospectionServer('server_name', self.sm, '/SM_TEST')
        self.sis.start()
        self.serv = rospy.Service("/start_SMACH", Empty, self.go)

        # Execute SMACH plan


        
    def go(self, req):
        
        print "Starting SMACH"
        locations = rospy.get_param('locations')
        self.sm.userdata.pose = locations[0]
        #self.sm.userdata.suffix = "_grip"
        self.sm.execute()
        return EmptyResponse()

    def stop(self):
        self.sis.stop()
        
if __name__ == '__main__':
    smach = SmachCBT()
    rospy.spin()
    smach.stop()



