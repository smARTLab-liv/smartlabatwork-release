from slaw_smach.DecisionStates import GetNextObjectLocationState
import rospy
from slaw_srvs.srv import GetObjectAtLocation, GetObjectAtLocationRequest

if __name__ == '__main__':
    rospy.init_node("object_manager_test")
    # get_best_at_location = rospy.ServiceProxy("/vision/object_manager/get_best", GetObjectAtLocation)
    #
    # best = None
    # req = GetObjectAtLocationRequest()
    # req.loc = 'S1'
    # req.side = 'left'
    # req.objects_to_pick = ['S40_40_B', 'S40_40_G']
    # try:
    #     rospy.wait_for_service("/vision/object_manager/get_best")
    #     best = get_best_at_location(req)
    # except rospy.ServiceException, e:
    #     print "Service call failed: %s" % e
    get_next_object = GetNextObjectLocationState()
    userdata = lambda: 0
    userdata.pose_in = {'name': 'S1'}
    userdata.pose_out = {'name': 'S1'}
    print get_next_object.execute(userdata)

    rospy.spin()