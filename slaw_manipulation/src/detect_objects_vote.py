#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from slaw_msgs.msg import PoseStampedLabeled

detected_objects = []
objects = {}
MAX_LIST_LENGTH = 20
THRESHOLD = 16

def object_voter(msg):
    pub = rospy.Publisher('/vision/detected_object_vote', String)

    # add the detected object to the list
    detected_objects.insert(0, msg.label)
    # if the object has not been detected before, add it to the objects dictionary
    if not msg.label in objects.keys():
        objects[msg.label] = 0
    # increase the count of the detected object
    objects[msg.label] = objects[msg.label] + 1

    # only keep MAX_LIST_LENGTH objects in the list
    if len(detected_objects) > MAX_LIST_LENGTH:
        # pop the oldest object from the list and decrease its counter
        popped_object = detected_objects.pop()
        objects[popped_object] = objects[popped_object] - 1

    # sort the list to find the most likely object
    sorted_objects = sorted(objects.items(), key=lambda x:x[1])

    # if the object is detected in the last MAX_LIST_LENGTH frames more than THRESHOLD times
    # publish the label of this object
    if sorted_objects[-1][1] > THRESHOLD:
        object_vote = sorted_objects[-1][0]
    # otherwise publish unknown
    else:
        object_vote = 'unknown'

    # publish the vote
    pub.publish(object_vote)

def listener():
    rospy.init_node('detected_object_voter')
    rospy.Subscriber('vision/detected_object', PoseStampedLabeled, object_voter)
    rospy.spin()
        
if __name__ == '__main__':
    listener()
