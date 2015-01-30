#!/usr/bin/env python
import rospy
import socket

from std_msgs.msg import String


def ros_init():
    rospy.init_node('refbox_parser')
    global sock, broadcastAddr

    broadcastAddr = (
        rospy.get_param("/refbox_updater/Braodcast_ADDR"), rospy.get_param("/refbox_updater/Braodcast_PORT"))
    rospy.loginfo("BradcastAdd: " + str(broadcastAddr))
    rospy.Subscriber(rospy.get_param("/refbox_updater/status_topic"), String, cb_update)

    host = ''  # Bind to all interfaces
    port = rospy.get_param("/refbox_updater/local_port")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind((host, port))

    rospy.spin()


def cb_update(status):
    global sock, broadcastAddr
    rospy.loginfo(status.data)
    sock.sendto("STATUS<" + str(status.data) + ">", broadcastAddr)


if __name__ == '__main__':
    ros_init()
