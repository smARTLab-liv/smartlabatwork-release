#!/usr/bin/env python
import rospy
import tf

import math
import numpy as np
from slaw_msgs.msg import PoseStampedLabeledArray
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseStamped, PoseArray
from slaw_srvs.srv import SwitchOnForLocation, SwitchOnForLocationResponse, GetObjectAtLocation, \
    GetObjectAtLocationResponse, RemoveObjectAtLocation, RemoveObjectAtLocationResponse
from collections import Counter

MIN_COUNT_PERC = 0.2
arm_frame = '/arm_base_link'
#global_frame = '/odom'
global_frame = '/map'
ALLOWED_DIST = 0.03

MAX_COUNTS = 50
MAX_OBJECTS = 10

MIN_RATE = 3
MIN_COUNT = 20

WINDOW = 5

VISIBLE_DIST = 0.18
EPS = 0.02

# arm_min = PoseStamped()
# arm_min.header.frame_id = arm_frame
# arm_min.header.stamp = rospy.Time(0)
# arm_min.pose.position.y = -VISIBLE_DIST
# arm_min.pose.orientation.w = 1.

# arm_max = PoseStamped()
# arm_max.header.frame_id = arm_frame
# arm_max.header.stamp = rospy.Time(0)
# arm_max.pose.position.y = VISIBLE_DIST
# arm_max.pose.orientation.w = 1.


BETTER_COLOR = ['M20_h', 'RV20_h']

merged_map = {'RV20': ['RV20_v', 'RV20_h'],
              'F20_20': ['F20_20_v'],
              'S40_40': ['S40_40_v']
}

objects_map = {'R20': ['RV20_h', 'RV20_v'],
               'V20': ['RV20_h', 'RV20_v'],
               'M20': ['M20_h', 'M20_v'],
               'M30': ['M30_h', 'M30_v'],
               'M20_100': ['M20_100_h', 'M20_100_HU_v', 'M20_100_HD_v'],
               'F20_20_G': ['F20_20_G_h', 'F20_20_G_v', 'F20_20_v'],
               'F20_20_B': ['F20_20_B_h', 'F20_20_B_v', 'F20_20_v'],
               'S40_40_G': ['S40_40_G_h', 'S40_40_G_v', 'S40_40_v'],
               'S40_40_B': ['S40_40_B_h', 'S40_40_B_v', 'S40_40_v']
}


class ObjectManager(object):
    def __init__(self):
        self.tfListen = tf.TransformListener()
        # listen for some time
        rospy.sleep(1)
        self.objects_at_location = {}
        self.active = False
        self.cur_loc = None
        self.setup_subscribers()

    def setup_subscribers(self):
        rospy.Subscriber('/vision/detected_objects', PoseStampedLabeledArray, self.cb_objects)

        rospy.Service('/vision/object_manager/switch_off', Empty, self.switch_off)

        rospy.Service('/vision/object_manager/switch_on', SwitchOnForLocation, self.switch_on_at_location)
        rospy.Service('/vision/object_manager/get_best', GetObjectAtLocation, self.get_best_object)
        rospy.Service('/vision/object_manager/verify_object', GetObjectAtLocation, self.verify_object)
        rospy.Service('/vision/object_manager/remove_all_objects', RemoveObjectAtLocation,
                      self.delete_all_objects_from_location)

        rospy.Service('/vision/object_manager/reset', Empty, self.reset)

        rospy.Service('/vision/object_manager/remove_object', RemoveObjectAtLocation, self.delete_object_from_location)
        rospy.Service('/vision/object_manager/blacklist_object', RemoveObjectAtLocation,
                      self.blacklist_object_from_location)

        self.pub = rospy.Publisher('/vision/objects', PoseArray)

    def reset(self, req):
        if self.active:
            self.switch_off(req)
        self.objects_at_location.clear()
        return EmptyResponse()

    def get_mapped_object(self, obj):
        for o in merged_map.keys():
            if obj in merged_map[o]:
                return o
        for o in objects_map.keys():
            if obj in objects_map[o]:
                return o

    def get_object_map(self, obj):
        return objects_map[obj]

    def cb_objects(self, msg):
        if not self.active:
            return
        for obj in msg.poses:
            if obj.pose.pose.position.y < -VISIBLE_DIST - EPS or obj.pose.pose.position.y > VISIBLE_DIST + EPS:
                # print 'not using', obj
                continue
            self.update_pose(obj)

        self.decrease_counters(msg.header.stamp, msg.detection_type)

        # sort by last seen and  remove oldest to get to max_objects
        self.objects_at_location[self.cur_loc].sort(key=lambda x: x['counter'])

        if len(self.objects_at_location[self.cur_loc]) > 0:
            while self.objects_at_location[self.cur_loc][0]['counter'] <= -5:
                remove = self.objects_at_location[self.cur_loc].pop(0)
                # print 'removed ', self.get_best_labels(remove)

        while len(self.objects_at_location[self.cur_loc]) > MAX_OBJECTS:
            self.objects_at_location[self.cur_loc].pop(0)
        msg_array = PoseArray()
        msg_array.header.stamp = rospy.Time.now()
        msg_array.header.frame_id = global_frame

        for obj in self.objects_at_location[self.cur_loc]:
            msg_array.poses.append(self.get_average_pose(obj).pose)
        self.pub.publish(msg_array)


    def transform_to_global(self, pose):
        try:
            tin = pose.header.stamp
            pose.header.stamp = rospy.Time(0)  # tin - rospy.Duration(0.05)
            # #self.tfListen.waitForTransform(arm_frame, pose.header.frame_id, pose.header.stamp, rospy.Duration(0.8))
            result = self.tfListen.transformPose(global_frame, pose)
            result.header.stamp = tin
            pose.header.stamp = tin
            return result
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print "transform exception"
            print e
            return None


    def transform_to_arm(self, pose):
        try:
            tin = pose.header.stamp
            pose.header.stamp = rospy.Time()
            # pose.header.stamp=tin - rospy.Duration(0.05)
            # self.tfListen.waitForTransform(arm_frame, pose.header.frame_id, pose.header.stamp, rospy.Duration(0.8))

            result = self.tfListen.transformPose(arm_frame, pose)
            result.header.stamp = tin
            pose.header.stamp = tin
            return result
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "transform exception"
            return None


    def decrease_counters(self, time, det_type):
        # base_min = self.transform_to_global(arm_min)
        # base_max = self.transform_to_global(arm_max)
        # print VISIBLE_DIST
        for obj in self.objects_at_location[self.cur_loc]:
            if not det_type in obj['type']:
                continue
            # pose = PoseStamped()
            # pose.pose = obj['poses'][-1]
            # pose.header.stamp = obj['last_seen'][-1]
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
                    obj['counter'] -= 1


    def update_pose(self, obj):
        pose_in_global = self.transform_to_global(obj.pose)
        if pose_in_global is None:
            return
        idx = self.find_closest_object(self.cur_loc, pose_in_global.pose)
        if idx == -1:
            # #only allow new objects when seen in depth
            if obj.detection_type == 'rgb' and not obj.label in BETTER_COLOR:
                return
            idx = self.create_observation(self.cur_loc)
            # print 'create_object', obj.label, 'pose global', pose_in_global.pose.position
        self.add_observation(self.cur_loc, idx, pose_in_global, obj)


    def create_observation(self, loc):
        cur = {}
        cur['poses'] = []
        cur['poses_in_arm'] = []
        cur['last_seen'] = []
        cur['labels'] = []
        cur['counter'] = 0
        cur['type'] = []
        cur['blacklisted'] = False
        self.objects_at_location[loc].append(cur)

        return len(self.objects_at_location[loc]) - 1


    def add_observation(self, loc, idx, pose_in_global, obj):
        cur = self.objects_at_location[loc][idx]
        if cur['blacklisted']:
            return

        cur['poses'].append(pose_in_global.pose)
        cur['poses_in_arm'].append(obj.pose)
        cur['last_seen'].append(obj.pose.header.stamp)
        cur['labels'].append(obj.label)
        cur['type'].append(obj.detection_type)
        cur['counter'] += 1
        cur['counter'] = min(cur['counter'], MAX_COUNTS)
        if len(cur['poses']) > MAX_COUNTS:
            cur['poses'].pop(0)
            cur['labels'].pop(0)
            cur['last_seen'].pop(0)
        cur['rate'] = self.calc_update_rate(cur['last_seen'])

    def calc_update_rate(self, timestamps):
        rates = []
        for idx, r in enumerate(timestamps):
            if idx == len(timestamps) - 1:
                break

            dif = (timestamps[idx + 1] - r).to_sec()
            if dif == 0.:
                continue

            rates.append((timestamps[idx + 1] - r).to_sec())
        if len(rates) == 0:
            return 0

        return 1. / np.median(rates)

    def switch_on_at_location(self, req):
        if req.loc not in self.objects_at_location.keys():
            self.objects_at_location[req.loc] = []
        self.cur_loc = req.loc
        print 'switching on for req.loc', req.loc
        self.active = True
        return SwitchOnForLocationResponse()

    def switch_off(self, req):
        print 'switching off'

        self.active = False
        # self.cur_loc = None
        return EmptyResponse()

    def find_closest_object(self, loc, pose):
        for idx, obj in enumerate(self.objects_at_location[loc]):
            # if self.dist(obj['poses'][-1], pose) < ALLOWED_DIST:
            if len(obj['poses']) == 0:
                continue

            if self.dist(self.get_average_pose(obj).pose, pose) < ALLOWED_DIST:
                return idx
        return -1

    def blacklist_object_from_location(self, req):
        if req.loc in self.objects_at_location.keys():
            pose = self.transform_to_global(req.pose)
            if pose is None:
                print 'No transform'
                return RemoveObjectAtLocationResponse()

            idx = self.find_closest_object(req.loc, pose.pose)
            if not idx == -1:
                self.objects_at_location[req.loc][idx]['blacklisted'] = True

        return RemoveObjectAtLocationResponse()


    def delete_object_from_location(self, req):
        if req.loc in self.objects_at_location.keys():
            pose = self.transform_to_global(req.pose)
            if pose is None:
                print 'No transform'
                return RemoveObjectAtLocationResponse()

            idx = self.find_closest_object(req.loc, pose.pose)
            if not idx == -1:
                del self.objects_at_location[req.loc][idx]

        return RemoveObjectAtLocationResponse()

    def delete_all_objects_from_location(self, req):
        if req.loc in self.objects_at_location.keys():
            del self.objects_at_location[req.loc]
        return RemoveObjectAtLocationResponse()

    def print_objects_at_location(self, req):
        if req.loc in self.objects_at_location.keys():
            for obj in self.objects_at_location[req.loc]:
                print obj['labels']
                print self.get_best_labels(obj)
                print obj['rate']
                print obj['counter']
                pose = self.get_average_pose(obj)
                print self.transform_to_arm(pose).pose.position
            print '-' * 40


    def dist(self, a, b):
        return math.sqrt(pow(a.position.x - b.position.x, 2) + pow(a.position.y - b.position.y, 2))


    def verify_object(self, req):
        objects_to_pick = req.objects_to_pick
        found_objects = self.objects_at_location[req.loc]
        pose = PoseStamped()
        pose.pose.orientation.w = 1.
        pose.header.frame_id = arm_frame
        pose.header.stamp = rospy.Time(0)  # rospy.Time.now()-rospy.Duration(0.1)

        self.print_objects_at_location(req)

        pose = self.transform_to_global(pose)
        min_dist = 9999
        f_obj = None
        label = None
        best_label = None
        now = rospy.Time.now()
        for idx, obj in enumerate(found_objects):
            if obj['blacklisted']:
                continue
            dist = self.dist(obj['poses'][-1], pose.pose)
            if obj['last_seen'][-1] < now - rospy.Duration(1.5):
                continue
            # if obj['rate'] < MIN_RATE:
            #    continue
            if dist < min_dist:
                labels = self.get_best_labels(obj)
                lab = labels[0][0]
                found = False
                for o in objects_to_pick:
                    if lab in self.get_object_map(o):
                        found = True
                        label = o
                        break
                if found:
                    best_label = lab
                    min_dist = dist
                    f_obj = obj

        if f_obj is None:
            res = GetObjectAtLocationResponse()
            res.object.label = "None"
            return res

        candidates = self.get_candidates(label, found_objects)

        # p = PoseStamped()
        # p.pose = f_obj['poses'][-1]
        # p.header.stamp = rospy.Time.now()
        # p.header.frame_id = global_frame
        # pose = self.transform_to_arm(p)
        # f_obj['pose_in_arm'] = pose

        res = GetObjectAtLocationResponse()
        res.object.label = best_label
        res.object.object = self.get_mapped_object(best_label)
        res.object.candidates = len(candidates)
        res.object.confidence = labels[0][1] / float(len(f_obj['labels']))
        res.object.pose = f_obj['poses_in_arm'][-1]
        return res


    def get_best_object(self, req):
        # print req
        objects_to_pick = req.objects_to_pick
        found_objects = self.objects_at_location[req.loc]
        candidates = []
        self.print_objects_at_location(req)
        for o in objects_to_pick:
            res = self.get_candidates(o, found_objects)
            # cands = {}
            # cands['name'] = o
            # cands['objects'] = res
            for p in res:
                candidates.append(p)
        print '\n\n'
        print'unsorted'
        for c in candidates:
            print c['labels'][0]
        if req.side == 'front':
            candidates.sort(key=lambda x: x['pose_in_arm'].pose.position.y, reverse=True)
        else:
            candidates.sort(key=lambda x: x['pose_in_arm'].pose.position.y)

        print 'Sorted '
        for c in candidates:
            print c['labels'][0]

        if len(candidates) == 0:
            res = GetObjectAtLocationResponse()
            res.object.label = "None"
            return res
        res = GetObjectAtLocationResponse()

        res.object.pose = candidates[-1]['pose_in_arm']
        res.object.label = candidates[-1]['labels'][0]
        # print res
        print 'object found: ' + str(res.object.label)
        print '\n\n'
        return res

    def get_candidates(self, obj, found_objects):
        res = []

        for f_obj in found_objects:
            if f_obj['blacklisted'] or f_obj['rate'] < MIN_RATE or f_obj['counter'] < MIN_COUNT:
                print str(f_obj['blacklisted']) + ' rate: ' + str(f_obj['rate']) + ' counter ' + str(
                    f_obj['counter']) + ' label: ' + str(f_obj['labels'][0])
                continue
            labels = self.get_best_labels(f_obj)

            for l, c in labels:
                if c < MIN_COUNT_PERC * len(f_obj['labels']):
                    print 'c: ' + str(c)
                    # continue
                print ' label: ' + str(l)
                if l in self.get_object_map(obj):
                    p = PoseStamped()
                    p.pose = f_obj['poses'][-1]
                    p.header.stamp = rospy.Time.now()
                    p.header.frame_id = global_frame
                    pose = self.transform_to_arm(p)
                    f_obj['pose_in_arm'] = pose
                    res.append(f_obj)
        return res


    def get_average_pose(self, found_object):
        pose = PoseStamped()
        x = []
        y = []
        z = []
        last_obj = min(len(found_object['poses']), WINDOW)
        for p in found_object['poses'][-last_obj:]:
            x.append(p.position.x)
            y.append(p.position.y)
            z.append(p.position.z)
        pose.header.stamp = found_object['last_seen'][-1]
        pose.header.frame_id = global_frame
        pose.pose.position.x = np.mean(x)
        pose.pose.position.y = np.mean(y)
        pose.pose.position.z = np.mean(z)
        return pose


    def get_best_labels(self, found_object):
        return Counter(found_object['labels']).most_common(2)


if __name__ == '__main__':
    rospy.init_node("object_manager")
    manager = ObjectManager()


    # req = lambda:0
    # req.loc = 'test'
    # manager.switch_on_at_location(req)
    # req.objects_to_pick = [ 'F20_20_G', 'M20_100']
    # print 'recording'
    # rospy.sleep(2)

    #manager.print_objects_at_location(req)
    #print manager.get_best_object(req)
    #print manager.verify_object(req)
    rospy.spin()
