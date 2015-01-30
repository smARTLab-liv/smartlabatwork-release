#!/usr/bin/env python
import rospy
from slaw_msgs.msg import SysInfo, NetworkStatus, SystemStatus
from pr2_msgs.msg import PowerBoardState
from random import randint

def talker():
    pub = rospy.Publisher('dashboard/sysinfo', SysInfo)
    rospy.init_node('broadcaster')

    msg = SysInfo()
    msg.network = NetworkStatus()
    msg.network.wifi_signallevel = -80
    msg.network.ethernet_connected = False

    msg.system = SystemStatus()
    msg.system.cpu_usage_average = randint(10,100)
    msg.system.cpu_temp_average = randint(10,100)
    
    pub.publish(msg)

def talker2():
    pub = rospy.Publisher('dashboard/platform_state', PowerBoardState)
    rospy.init_node('broadcaster')

    msg = PowerBoardState()
    msg.circuit_state = [PowerBoardState.STATE_ENABLED, PowerBoardState.STATE_STANDBY, PowerBoardState.STATE_ENABLED]
    
    pub.publish(msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
