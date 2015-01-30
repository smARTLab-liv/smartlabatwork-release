#!/usr/bin/env python
import rospy
# import tf

from slaw_msgs.msg import PoseStampedLabeledArray

from std_srvs.srv import Empty
from geometry_msgs.msg import PoseArray
from slaw_srvs.srv import SwitchOnForLocation, GetObjectAtLocation, RemoveObjectAtLocation

from object_manager import ObjectManager, global_frame

VISIBLE_DIST = 0.13
EPS = 0.02
MAX_HOLES = 10


class HolesManager(ObjectManager):
    def __init__(self):
        super(HolesManager, self).__init__()
        #self.setup_subscribers()

    def setup_subscribers(self):
        rospy.Subscriber('/vision/detected_holes', PoseStampedLabeledArray, self.cb_holes)

        rospy.Service('/vision/hole_manager/switch_off', Empty, self.switch_off)
        rospy.Service('/vision/hole_manager/reset', Empty, self.reset)

        rospy.Service('/vision/hole_manager/switch_on', SwitchOnForLocation, self.switch_on_at_location)
        rospy.Service('/vision/hole_manager/get_best', GetObjectAtLocation, self.get_best_object)
        rospy.Service('/vision/hole_manager/verify_hole', GetObjectAtLocation, self.verify_object)
        rospy.Service('/vision/hole_manager/remove_all_holes', RemoveObjectAtLocation,
                      self.delete_all_objects_from_location)
        rospy.Service('/vision/hole_manager/remove_hole', RemoveObjectAtLocation, self.delete_object_from_location)
        rospy.Service('/vision/hole_manager/blacklist_hole', RemoveObjectAtLocation,
                      self.blacklist_object_from_location)

        self.pub = rospy.Publisher('/vision/holes', PoseArray)
        print 'Hole services setup'

    def get_mapped_object(self, obj):
        return obj

    def get_object_map(self, obj):
        return [obj]

    def update_pose(self, obj):
        pose_in_global = self.transform_to_global(obj.pose)
        if pose_in_global is None:
            return
        idx = self.find_closest_object(self.cur_loc, pose_in_global.pose)
        if idx == -1:
            ##only allow new objects when seen in depth
            idx = self.create_observation(self.cur_loc)
            #print 'create_object', obj.label, 'pose global', pose_in_global.pose.position
        self.add_observation(self.cur_loc, idx, pose_in_global, obj)


    def cb_holes(self, msg):
        if not self.active:
            return
        for obj in msg.poses:
            if obj.pose.pose.position.y < -VISIBLE_DIST - EPS or obj.pose.pose.position.y > VISIBLE_DIST + EPS:
                #print 'not using', obj
                continue
            self.update_pose(obj)

        self.decrease_counters(msg.header.stamp, msg.detection_type)

        #sort by last seen and  remove oldest to get to max_objects
        self.objects_at_location[self.cur_loc].sort(key=lambda x: x['counter'])

        if len(self.objects_at_location[self.cur_loc]) > 0:
            while self.objects_at_location[self.cur_loc][0]['counter'] <= -5:
                remove = self.objects_at_location[self.cur_loc].pop(0)
                #print 'removed ', self.get_best_labels(remove)

        while len(self.objects_at_location[self.cur_loc]) > MAX_HOLES:
            self.objects_at_location[self.cur_loc].pop(0)
        msg_array = PoseArray()
        msg_array.header.stamp = rospy.Time.now()
        msg_array.header.frame_id = global_frame

        for obj in self.objects_at_location[self.cur_loc]:
            msg_array.poses.append(self.get_average_pose(obj).pose)
        self.pub.publish(msg_array)


    def decrease_counters(self, time, det_type):
        #base_min = self.transform_to_global(arm_min)
        #base_max = self.transform_to_global(arm_max)

        for obj in self.objects_at_location[self.cur_loc]:
            if not det_type in obj['type']:
                continue
            #pose = PoseStamped()
            #pose.pose = obj['poses'][-1]
            #pose.header.stamp = obj['last_seen'][-1]
            #pose.header.frame_id = global_frame
            pose = self.get_average_pose(obj)
            pose = self.transform_to_arm(pose)
            if pose is None:
                continue
            pose = pose.pose.position

            if pose.y < -VISIBLE_DIST or pose.y > VISIBLE_DIST:
                continue
            else:
                if not obj['last_seen'][-1] == time:
                    #print pose, obj['labels'][-1]
                    #print 'decrease', obj['labels'][-1], obj['counter']
                    obj['counter'] -= 1


if __name__ == '__main__':
    rospy.init_node("hole_manager")
    manager = HolesManager()


    # req = lambda:0
    # req.loc = 'test'
    # manager.switch_on_at_location(req)
    # #req.objects_to_pick = [ 'F20_20_G', 'M20_100']
    # print 'recording'
    # rospy.sleep(2)
    #
    # manager.print_objects_at_location(req)
    #print manager.get_best_object(req)
    #print manager.verify_object(req)
    rospy.spin()
