#!/usr/bin/env python
from smach_helpers import get_objects_on_plate

import rospy
import tf
import math
import actionlib
import smach
import copy
import numpy as np
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty
from slaw_actions.msg import *
from slaw_srvs.srv import *
from slaw_msgs.msg import *
from geometry_msgs.msg import Pose
from arm_helpers import grip_point, init_arm_helpers, call_ik_solver, get_place_height_for_obj
from smach_helpers import get_objects_on_plate, backplate_object_msg_to_dict, publish_status
from slaw_srvs.srv import CanPlaceObject, CanPlaceObjectResponse


class ChooseTargetPositionManipulation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'],
                             output_keys=['pose_out'])
        self.can_place_object = rospy.ServiceProxy("/backplate_manager/can_place_object", CanPlaceObject)
        self.last_chosen_position = None
        self.counter = 0

    def get_objects_for_location(self, loc_name):
        return rospy.get_param(loc_name)['objects']


    def get_pickup_location_to_object_map(self):
        locations_param = rospy.get_param('locations')
        locations = {}
        for loc in locations_param:
            if loc['type'] == 'Pickup':
                obj = self.get_objects_for_location(loc['name'])
                if len(obj) > 0:
                    locations[loc['name']] = obj
        return locations

    def get_location_with_most_obj_to_place(self, objects_on_plate):
        max_items_to_place = 0
        max_loc = None
        location_counters = {}

        for o in objects_on_plate:
            if o.destination_loc not in location_counters:
                location_counters[o.destination_loc] = 1
            else:
                location_counters[o.destination_loc] += 1

            if location_counters[o.destination_loc] > max_items_to_place:
                max_loc = o.destination_loc
                max_items_to_place = location_counters[o.destination_loc]

        return max_loc

    def get_end_locarion(self):
        locations_param = rospy.get_param('locations')
        res = {}
        res['name'] = 'D0'
        res['type'] = 'End'

        for loc in locations_param:
            if loc['type'] == 'End':
                res['name'] = loc['name']
                res['type'] = 'End'
        return res

    def execute(self, userdata):
        objects_on_plate = get_objects_on_plate()
        pickup_location_object_map = self.get_pickup_location_to_object_map()
        # sort after length of objects
        pickup_locs_sorted = sorted(pickup_location_object_map, key=lambda k: len(pickup_location_object_map[k]),
                                    reverse=True)

        # toDo loc up location with type end
        best_pickup_loc = self.get_end_locarion()

        items_on_plate = len(objects_on_plate)

        if items_on_plate == 0 and len(pickup_location_object_map) > 0:
            # location with most items to pic
            best_pickup_loc['name'] = pickup_locs_sorted[0]
            best_pickup_loc['type'] = 'Pickup'
            publish_status("I am going to " + str(best_pickup_loc['name']) + ' to pickup objects.')
            print "Pickup, since nothing on plate"
        # place items
        elif items_on_plate == 3 or (len(pickup_location_object_map) == 0 and items_on_plate > 0):
            best_pickup_loc['name'] = self.get_location_with_most_obj_to_place(objects_on_plate)
            best_pickup_loc['type'] = 'Place'
            publish_status(
                "My back plate is full, I am going to " + str(best_pickup_loc['name']) + ' to place objects.')
            print "Place, since plate full or nothing left to pick"

        #choose extra item to pick
        else:
            found = False
            for loc_key in pickup_locs_sorted:
                objects_to_pickup = pickup_location_object_map[loc_key]
                if len(objects_to_pickup) <= 3 - items_on_plate:
                    found = True
                    for obj in objects_to_pickup:
                        req = self.can_place_object(obj['name'])
                        if not req.can_place:
                            found = False
                            break
                    if found:
                        best_pickup_loc['name'] = loc_key
                        best_pickup_loc['type'] = 'Pickup'
                        publish_status("I am going to " + str(best_pickup_loc['name']) + '<br> to pickup objects.')
                        print "Pickup, can kill other platform"

            if not found and items_on_plate > 0:
                best_pickup_loc['name'] = self.get_location_with_most_obj_to_place(objects_on_plate)
                best_pickup_loc['type'] = 'Place'
                publish_status("I am going to " + str(best_pickup_loc['name']) + '<br> to place objects.')
                print "place since no other platform to kill"

        if best_pickup_loc['type'] == 'End':
            print 'Go To end'
        userdata.pose_out = best_pickup_loc
        if self.last_chosen_position is not None and self.last_chosen_position['name'] == \
                best_pickup_loc['name']:
            self.counter += 1
            if self.counter > 2:
                # delete location from param server
                locations_param = rospy.get_param('locations')
                for loc in locations_param:
                    if loc['name'] == self.last_chosen_position['name']:
                        locations_param.remove(loc)
                        break
                rospy.set_param('locations', locations_param)
                self.counter = 0
        else:
            self.counter = 0
        self.last_chosen_position = best_pickup_loc
        return 'done'
