import rospy
import math
import numpy as np
import actionlib
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from slaw_actions.msg import GripOrPlaceGoal, GripOrPlaceAction
from slaw_srvs.srv import SimpleIkSolver, SimpleIkSolverRequest
from trajectory_msgs.msg import JointTrajectoryPoint
from smach_helpers import init_helpers


pre_kinect = [5.323699090192345, 1.3339887113233428, -2.446201119717692, 1.8014157762386995, 2.9404422281923392]

pre_grip = [5.323699090192345 - math.pi, 1.134883 + 0.4, -4.119026326794897 + math.pi / 2.,
            3.3593047267948966 - math.pi / 2. - 0.4, 2.923]

base_frame = '/arm_base_link'
end_effector_frame = '/arm_tip_link'

joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]

joints = {}

grip_point = [0.25, 0.0, 0.0]  # 0.28 #0.15

# cam_test = [0.5666144288551437, 1.538595002095601, -1.3731744409208306, 1.7847786095098293, 2.898163470720438]
cam_test = [0.5797748826956625, 0.8453502232034533, -0.7155448507448792, 1.9841591235531482, 2.9311723174750575]
cam_test_drop = [0.1776812306578381, 1.1879952757964247, -0.8228930717180426, 1.5743140256112402, 2.872853738356306]

PRE_GRIP_HEIGHT = 0.08
PRE_PLACE_HEIGHT = 0.04

#TABLE_HEIGHT = 0.106
TABLE_HEIGHT = 0.109
ABOVE_TABLE = 0.002

STEP_SIZE = 0.04

MAX_TRIES = None
TIME_OUT = None
NO_ARM = None
arm_client = None
gripper_client = None
mount_offset = None
arm_base_link_height = None
iks = None

configuration = [0, 0, 0, 0, 0]
gotArmConf = False

CENTER_ANGLES = [0.6113, 0.3113, 0.03606]
NIPPLE_ANGLES = [0.765, 0.765]
BIG_HOLE_ANGLE = 0.405

HIGH_OFFSET = 0.4

PLATFORM_HEIGHT_OFFSET = 0.003


def init_arm_helpers():
    global MAX_TRIES, TIME_OUT, NO_ARM, arm_client, iks, \
        gripper_client, joints, mount_offset, arm_base_link_height, grip_point
    if MAX_TRIES is None:
        init_helpers()
        MAX_TRIES = rospy.get_param("smach/max_tries", 5)
        TIME_OUT = rospy.get_param("smach/time_out", 5.0)
        NO_ARM = rospy.get_param("smach/no_arm", False)

        joints = rospy.get_param("joints")
        mount_offset = rospy.get_param("arm_rot_offset")
        #Gripper client
        gripper_client = actionlib.SimpleActionClient('gripper_action', GripOrPlaceAction)
        arm_action_name = rospy.get_param('~arm_joint_trajectory_action',
                                          '/arm_1/arm_controller/joint_trajectory_action')
        arm_client = actionlib.SimpleActionClient(arm_action_name, FollowJointTrajectoryAction)

        iks = rospy.ServiceProxy('/arm_1/simple_ik_server', SimpleIkSolver)
        arm_base_link_height = rospy.get_param('arm_base_link_height')
        grip_point[2] = TABLE_HEIGHT + ABOVE_TABLE - arm_base_link_height

        if not NO_ARM:
            rospy.Subscriber('/joint_states', JointState, joint_state_cb)

            if not gripper_client.wait_for_server(rospy.Duration(TIME_OUT)):
                rospy.logerr("gripper action server did not come up within timelimit")
            if not arm_client.wait_for_server(rospy.Duration(TIME_OUT)):
                rospy.logerr("arm_joint_client action server did not come up within timelimit")

        rospy.loginfo("globals initialized")
        rospy.sleep(0.5)
    else:
        rospy.loginfo("globals already initialized")


def get_place_height_for_obj(object_label):
    obj = rospy.get_param(object_label)
    return obj['place_height_offset']


def get_pick_height_for_obj(object_label):
    obj = rospy.get_param(object_label)
    return obj['pick_height_offset']


def get_platform_place_height_for_obj(object_label):
    obj = rospy.get_param(object_label)
    return obj['platform_place_height_offset']


def get_platform_pick_height_for_obj(object_label):
    obj = rospy.get_param(object_label)
    return obj['platform_pick_height_offset']


def call_ik_solver(goal_point, side='front'):
    req = SimpleIkSolverRequest()
    req.position = side

    req.point = PointStamped()
    req.point.point.x = goal_point[0]
    req.point.point.y = goal_point[1]
    req.point.point.z = goal_point[2]

    req.point.header.frame_id = base_frame

    req.point.header.stamp = rospy.Time()
    resp = None
    try:
        resp = iks(req)
    except rospy.ServiceException, e:
        rospy.logerr("Service did not process request: %s", str(e))
    if resp is not None:
        return resp.joints
    return None


def go(positions, move_duration=2.5, blocking=True):
    if NO_ARM:
        rospy.sleep(1.0)
        return True

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [x for x in joint_names]
    goal.trajectory.points = []
    for p, count in zip(positions, range(0, len(positions) + 1)):
        goal.trajectory.points.append(JointTrajectoryPoint(positions=p,
                                                           velocities=[],
                                                           accelerations=[],
                                                           time_from_start=rospy.Duration((count + 1) * move_duration)))

        goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)

    if blocking:
        arm_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))
        res = arm_client.get_result()

        if arm_client.get_state() == GoalStatus.SUCCEEDED:
            return True

        rospy.logerr("Arm failed!!!")
        return False
    else:
        arm_client.send_goal(goal)
        return True



def confs_for_platform_lower_center(place=1, rot_offset=0., height_offset=0.):
    point = [0.275, 0.0, -0.015 + height_offset + PLATFORM_HEIGHT_OFFSET]
    point1 = [0.27, 0.0, 0.0 + height_offset + PLATFORM_HEIGHT_OFFSET]
    point2 = [0.27, 0.0, 0.02]
    point3 = [0.21, 0.0, 0.06]
    conf0 = np.array(call_ik_solver(point, side='back'))
    conf1 = np.array(call_ik_solver(point1, side='back'))
    conf2 = np.array(call_ik_solver(point2, side='back'))
    conf3 = np.array(call_ik_solver(point3, side='back'))
    conf3[3] -= HIGH_OFFSET
    conf3[1] -= HIGH_OFFSET
    conf4 = [x for x in conf3]
    conf2[3] -= HIGH_OFFSET
    conf2[1] -= HIGH_OFFSET
    confs = [conf4, conf3, conf2, conf1, conf0]
    for con in confs:
        con[0] = CENTER_ANGLES[place - 1]
        con[4] += rot_offset
    conf4[0] = configuration[0]
    return confs

## TODO check for height of offsets!!!
def confs_for_platform_center(place=1, rot_offset=0., height_offset=0.):
    point = [0.305, 0.0, -0.015 + height_offset + PLATFORM_HEIGHT_OFFSET]
    point1 = [0.295, 0.0, 0.0 + height_offset + PLATFORM_HEIGHT_OFFSET]
    point2 = [0.27, 0.0, 0.02 + height_offset]
    point3 = [0.21, 0.0, 0.06]
    conf0 = np.array(call_ik_solver(point, side='back'))
    conf1 = np.array(call_ik_solver(point1, side='back'))
    conf2 = np.array(call_ik_solver(point2, side='back'))
    conf3 = np.array(call_ik_solver(point3, side='back'))
    conf3[3] -= HIGH_OFFSET
    conf3[1] -= HIGH_OFFSET
    conf4 = [x for x in conf3]
    conf2[3] -= HIGH_OFFSET
    conf2[1] -= HIGH_OFFSET
    confs = [conf4, conf3, conf2, conf1, conf0]
    for con in confs:
        con[0] = CENTER_ANGLES[place - 1]
        con[4] += rot_offset
    conf4[0] = configuration[0]
    return confs


def confs_for_platform_big_hole(rot_offset=0., height_offset=0.):
    point = [0.235, 0.0, -0.015 + height_offset + PLATFORM_HEIGHT_OFFSET]
    point1 = [0.235, 0.0, 0.0 + height_offset + PLATFORM_HEIGHT_OFFSET]
    point2 = [0.24, 0.0, 0.02 + height_offset]
    point3 = [0.21, 0.0, 0.06]

    conf0 = np.array(call_ik_solver(point, side='back'))
    conf1 = np.array(call_ik_solver(point1, side='back'))
    conf2 = np.array(call_ik_solver(point2, side='back'))
    conf3 = np.array(call_ik_solver(point3, side='back'))
    conf3[3] -= HIGH_OFFSET
    conf3[1] -= HIGH_OFFSET
    conf4 = [x for x in conf3]
    conf2[3] -= HIGH_OFFSET
    conf2[1] -= HIGH_OFFSET
    confs = [conf4, conf3, conf2, conf1, conf0]
    for con in confs:
        con[0] = BIG_HOLE_ANGLE
        con[4] += rot_offset
    conf4[0] = configuration[0]

    return confs


def confs_for_platform_nipple(place=1, rot_offset=0., height_offset=0.):

    point = [0.305, 0.0, -0.015 + height_offset + PLATFORM_HEIGHT_OFFSET]
    point1 = [0.30, 0.0, 0.0 + height_offset + PLATFORM_HEIGHT_OFFSET]
    point2 = [0.24, 0.0, 0.02 + height_offset]
    point3 = [0.21, 0.0, 0.06]
    if place == 2:
        point = [0.235, 0.0, -0.015 + height_offset + PLATFORM_HEIGHT_OFFSET]
        point1 = [0.235, 0.0, 0.0 + height_offset + PLATFORM_HEIGHT_OFFSET]
        point2 = [0.23, 0.0, 0.02 + height_offset]


    conf0 = np.array(call_ik_solver(point, side='back'))
    conf1 = np.array(call_ik_solver(point1, side='back'))
    conf2 = np.array(call_ik_solver(point2, side='back'))
    conf3 = np.array(call_ik_solver(point3, side='back'))
    conf3[3] -= HIGH_OFFSET
    conf3[1] -= HIGH_OFFSET
    conf4 = [x for x in conf3]
    conf2[3] -= HIGH_OFFSET
    conf2[1] -= HIGH_OFFSET
    confs = [conf4, conf3, conf2, conf1, conf0]
    for con in confs:
        con[0] = NIPPLE_ANGLES[place - 1]
        con[4] += rot_offset
    conf4[0] = configuration[0]

    return confs

def place_on_platform(object, backplate_pose):
    # places_names 'center_x','lower_center_x', 'center_x_turned', nippel_x, big_hole_1
    rot_offset = 0.0
    # # TODO: Add more locations
    s = backplate_pose.split('_')
    if "turned" in backplate_pose:
        rot_offset = math.pi / 2.0
    height_offset = get_platform_place_height_for_obj(object['label'])
    confs = None
    if "center" == s[0]:
        pos = int(s[1])
        confs = confs_for_platform_center(pos, rot_offset=rot_offset, height_offset=height_offset)
    elif "lower" == s[0]:
        rot_offset = math.pi / 2.0
        pos = int(s[2])
        confs = confs_for_platform_lower_center(pos, rot_offset=rot_offset, height_offset=height_offset)
    elif "nippel" == s[0]:
        rot_offset = math.pi / 2.0
        pos = int(s[2])
        confs = confs_for_platform_nipple(pos, rot_offset=rot_offset, height_offset=height_offset)
    elif "big_hole" in backplate_pose:
        rot_offset = 0
        # pos = int(s[2])
        confs = confs_for_platform_nipple(rot_offset=rot_offset, height_offset=height_offset)

    go(confs)
    rospy.sleep(0.5)
    gripper_close(False, width=0.25)
    go([confs[-2], confs[1]])


def pick_from_platform(object, backplate_pose):
    gripper_close(False, width=0.25)
    heavy = False

    if object['label'] == "M20_100_h":
        heavy = True

    rot_offset = 0.0
    # # TODO: Add more locations
    s = backplate_pose.split('_')
    if "turned" in backplate_pose:
        rot_offset = math.pi / 2.0
    height_offset = get_platform_pick_height_for_obj(object['label'])
    confs = None
    if "center" == s[0]:
        pos = int(s[1])
        confs = confs_for_platform_center(pos, rot_offset=rot_offset, height_offset=height_offset)
    elif "lower" == s[0]:
        rot_offset = math.pi / 2.0
        pos = int(s[2])
        confs = confs_for_platform_lower_center(pos, rot_offset=rot_offset, height_offset=height_offset)
    elif "nippel" == s[0]:
        rot_offset = math.pi / 2.0
        pos = int(s[2])
        confs = confs_for_platform_nipple(pos, rot_offset=rot_offset, height_offset=height_offset)
    elif "big_hole" in backplate_pose:
        rot_offset = 0
        # pos = int(s[2])
        confs = confs_for_platform_nipple(rot_offset=rot_offset, height_offset=height_offset)
    go(confs)
    rospy.sleep(0.5)
    gripper_close(True, heavy=heavy)
    go([confs[-2], confs[1]])


def gripper_close(close_grip, heavy=False, width=None):
    grip = GripOrPlaceGoal()
    grip.close = close_grip
    grip.heavy = heavy
    if width is None:
        grip.set_point = False
    else:
        grip.set_point = True
        grip.width = width
    gripper_client.send_goal_and_wait(grip, rospy.Duration(30.0), rospy.Duration(5.0))


def rotate_only_angle(angle):
    rotate = [x for x in configuration]
    rotate[0] = angle
    return rotate


def get_angle(straight, angle):
    ang = straight - angle
    # print straight, angle
    while ang < straight - math.pi / 2:
        ang = ang + math.pi
    while ang > straight + math.pi / 2:
        ang = ang - math.pi
    return ang


def limit_joint1_ang(ang):
    while ang < joints['arm_joint_1']['min']:
        ang += 2 * math.pi
    while ang > joints['arm_joint_1']['max']:
        ang -= 2 * math.pi
    return ang


def rotate_only(side, turned=False):
    rotate = [x for x in configuration]
    straight = joints['arm_joint_1']['straight'] - mount_offset

    rotate[0] = straight

    if side == 'left':
        rotate[0] += -math.pi / 2
    elif side == 'right':
        rotate[0] += +math.pi / 2
    elif side == 'back':
        rotate[0] += math.pi

    if turned:
        rotate[0] = limit_joint1_ang(rotate[0] + math.pi)
    return rotate


def joint_state_cb(msg):
    global configuration, gotArmConf
    for k in range(5):
        for i in range(len(msg.name)):
            joint_name = "arm_joint_" + str(k + 1)
            if msg.name[i] == joint_name:
                configuration[k] = msg.position[i]
                gotArmConf = True