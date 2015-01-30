#!/usr/bin/env python
__author__ = 'youbot'

import rospy
from std_srvs.srv import Empty
from slaw_srvs.srv import CheckRV20, GetLocation

LOCATION = 'S11'

if __name__ == '__main__':
    rospy.init_node('testing')
    destination_manager = rospy.ServiceProxy('/smach/destination_manager_leave_location', Empty)
    try:
        res = destination_manager()
        print 'destination manager online', res
    except Exception as e:
        print 'destination manager test failed ', e

    rv20 = rospy.ServiceProxy('/vision/check_rv20', CheckRV20)
    try:
        res = rv20()
        print 'rv20 check ok, but check image', res
    except Exception as e:
        print 'rv20 check failed ', e

    loc_manager = rospy.ServiceProxy('/location_service/get_location', GetLocation)

    try:
        res = loc_manager(name=LOCATION)
        print 'location manager online'
        if len(res.locations) > 0:
            print 'I got a valid location for ' + LOCATION
        else:
            print 'location result for ' + LOCATION + ' is empty please check!'
    except Exception as e:
        print 'locataion manager test failed ', e
