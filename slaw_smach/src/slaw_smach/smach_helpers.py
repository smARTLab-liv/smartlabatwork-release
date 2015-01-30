from geometry_msgs.msg import PointStamped
import rospy
from slaw_msgs.msg import BackplateObject
from slaw_srvs.srv import GetLocation, BackplateStateStatus
from std_msgs.msg import String

loc_service = None
TIME_OUT = None
base_frame = '/arm_base_link'
get_objects_on_plate_srv = None
status_pub = None


def init_helpers():
    global TIME_OUT, loc_service, get_objects_on_plate_srv, status_pub
    if TIME_OUT is None:
        TIME_OUT = rospy.get_param("smach/time_out", 5.0)
        loc_service = rospy.ServiceProxy("location_service/get_location", GetLocation)
        get_objects_on_plate_srv = rospy.ServiceProxy("/backplate_manager/get_objects_from_plate",
                                                      BackplateStateStatus)


def publish_status(status_string):
    global status_pub
    if status_pub is None:
        status_pub = rospy.Publisher('/refbox/status', String)
    try:
        for i in xrange(3):
            status_pub.publish(status_string)
    except Exception as e:
        print e


def get_pickup_side(location):
    try:
        rospy.wait_for_service('location_service/get_location', timeout=TIME_OUT)
        loc = loc_service(location)  # string name
        return loc.locations[0].pickup_side
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        return 'failed'


def get_objects_on_plate():
    res = get_objects_on_plate_srv()
    return res.objects_on_plate


def dict_to_backplate_object_msg(dict):
    msg = BackplateObject()
    msg.label = dict['label']
    msg.obj_name = dict['name']
    msg.place_type = dict['type']
    msg.destination_loc = dict['destination_location']
    msg.place_loc = dict['backplate_location']
    if 'rotation' in dict.keys():
        msg.rotation = dict['rotation']
    return msg


def backplate_object_msg_to_dict(msg):
    obj = {}
    obj['name'] = msg.obj_name
    obj['label'] = msg.label
    obj['type'] = msg.place_type
    obj['destination_location'] = msg.destination_loc
    obj['backplate_location'] = msg.place_loc
    obj['rotation'] = msg.rotation

    return obj

