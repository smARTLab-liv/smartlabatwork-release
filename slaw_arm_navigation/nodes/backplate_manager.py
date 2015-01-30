#!/usr/bin/env python
import rospy
import math
import copy
from slaw_srvs.srv import GetBestPlaceLocation, GetBestPlaceLocationResponse, CanPlaceObject, CanPlaceObjectResponse, \
    BackplateStateModify, BackplateStateModifyResponse, BackplateStateStatus, BackplateStateStatusResponse, \
    BackplateStateModifyRequest, GetBestPlaceLocationRequest

from slaw_msgs.msg import BackplateObject
from std_srvs.srv import Empty, EmptyResponse

MAX_OBJECTS = 3

base_frame = '/arm_base_link'

objects_map = {'R20': ['R20_h', 'R20_v', 'RV20_h', 'RV20_v'],
               'V20': ['V20_h', 'V20_v', 'RV20_h', 'RV20_v'],
               'M20': ['M20_h', 'M20_v'],
               'M30': ['M30_h', 'M30_v'],
               'M20_100': ['M20_100_h', 'M20_100_HU_v', 'M20_100_HD_v'],
               'F20_20_G': ['F20_20_G_h', 'F20_20_G_v', 'F20_20_v'],
               'F20_20_B': ['F20_20_B_h', 'F20_20_B_v', 'F20_20_v'],
               'S40_40_G': ['S40_40_G_h', 'S40_40_G_v', 'S40_40_v'],
               'S40_40_B': ['S40_40_B_h', 'S40_40_B_v', 'S40_40_v']
}

interference_map = {'center_1': ['lower_center_1', 'center_1_turned', 'nippel_1'],
                    'center_2': ['lower_center_2', 'center_2_turned'],
                    'center_3': ['lower_center_3', 'center_3_turned'],
                    'lower_center_1': ['center_1', 'center_1_turned'],
                    'lower_center_2': ['center_2', 'center_2_turned'],
                    'lower_center_3': ['center_3', 'center_3_turned'],
                    'center_1_turned': ['center_1', 'lower_center_1'],
                    'center_2_turned': ['center_2', 'lower_center_2'],
                    'center_3_turned': ['center_3', 'lower_center_3'],
                    'nippel_1': ['center_1', 'center_1_turned'],
                    'nippel_2': ['big_hole_1'],
                    'big_hole_1': ['nippel_2']
}

place_options = {'M20_h': ['lower_center_1', 'lower_center_2', 'lower_center_3'],  # Nippels?
                 'M30_h': ['center_1', 'center_2', 'center_3'],
                 'M20_100_h': ['center_1', 'center_2', 'center_3'],
                 'F20_20_G_h': ['center_1', 'center_2', 'center_3'],
                 'F20_20_B_h': ['center_1', 'center_2', 'center_3'],
                 'S40_40_G_h': ['center_1', 'center_2', 'center_3'],
                 'S40_40_B_h': ['center_1', 'center_2', 'center_3'],
                 'R20_h': ['center_1', 'center_2', 'center_3'],
                 'V20_h': ['center_1', 'center_2', 'center_3'],
                 'RV20_h': ['center_1', 'center_2', 'center_3'],

                 # # Vertical objects
                 'M20_v': ['center_1', 'center_2', 'center_3'],
                 'M30_v': ['center_1_turned', 'center_2_turned', 'center_3_turned'],  # turned 90 degrees
                 'M20_100_HD_v': ['big_hole_1'],
                 'M20_100_HU_v': ['center_2'],
                 'F20_20_G_v': ['center_1', 'center_3', 'big_hole_1'],
                 'F20_20_B_v': ['center_1', 'center_3', 'big_hole_1'],
                 'F20_20_v': ['center_1', 'center_3', 'big_hole_1'],
                 'S40_40_v': ['nippel_2'],
                 'S40_40_G_v': ['nippel_2'],
                 'S40_40_B_v': ['nippel_2'],
                 'R20_v': ['nippel_1', 'nippel_2'],
                 'V20_v': ['nippel_1', 'nippel_2'],
                 'RV20_v': ['nippel_1', 'nippel_2']
}


class BackPlateManager:
    def __init__(self):
        self.objects_on_plate = []
        rospy.Service('/backplate_manager/can_place_object', CanPlaceObject, self.can_place_object_all_rotations)
        rospy.Service('/backplate_manager/can_place_specific_object', CanPlaceObject, self.can_place_specific_object)
        rospy.Service('/backplate_manager/add_object_to_plate', BackplateStateModify, self.add_object_to_plate)
        rospy.Service('/backplate_manager/remove_object_from_plate', BackplateStateModify,
                      self.remove_object_from_plate)
        rospy.Service('/backplate_manager/get_objects_from_plate', BackplateStateStatus, self.get_objects_from_plate)
        rospy.Service('/backplate_manager/get_best_place_location', GetBestPlaceLocation, self.get_best_place_location)

        rospy.Service('/backplate_manager/remove_all_objects_from_plate', Empty, self.remove_all_objects_from_plate)

        # print back_plate


    def can_place_specific_object(self, req):
        obj = req.obj_name
        options = self.get_current_options(obj)
        if len(options) == 0:
            return CanPlaceObjectResponse(can_place=False)
        return CanPlaceObjectResponse(can_place=True)


    def can_place_object_all_rotations(self, req):
        obj = req.obj_name
        for o in objects_map[obj]:
            options = self.get_current_options(o)
            if len(options) == 0:
                return CanPlaceObjectResponse(can_place=False)
        return CanPlaceObjectResponse(can_place=True)

    def get_objects_from_plate(self, req):
        return BackplateStateStatusResponse(objects_on_plate=self.objects_on_plate)

    def add_object_to_plate(self, req):
        self.objects_on_plate.append(req.obj)
        return BackplateStateModifyResponse()

    def remove_all_objects_from_plate(self, req):
        #for obj in self.objects_on_plate:
        #    self.objects_on_plate.pop()
        self.objects_on_plate = []
        return EmptyResponse()

    def remove_object_from_plate(self, req):
        for index, obj in enumerate(self.objects_on_plate):
            if obj.label == req.obj.label and obj.obj_name == req.obj.obj_name and obj.place_type == req.obj.place_type \
                    and obj.place_loc == req.obj.place_loc:
                self.objects_on_plate.pop(index)
                break
        return BackplateStateModifyResponse()

    # # Get location with the least interfearances
    def get_best_place_location(self, req):
        obj = req.obj_name
        objects_to_pick = req.objects_to_pick
        cur_obj = self.get_mapped_object(obj)
        objects_to_pick.remove(cur_obj)
        print objects_to_pick
        possible_places = place_options[obj]
        used = self.get_unavailable_places()
        possible_places = self.get_current_options(obj, used)

        if len(possible_places) == 0:
            return GetBestPlaceLocationResponse(place_loc='')

        obj_without_place = [0] * len(possible_places)
        for idx, place in enumerate(possible_places):
            for o_ in objects_to_pick:
                for o in objects_map[o_]:
                    options = self.get_current_options(o, used)
                    if place in options and len(options) == 1:
                        obj_without_place[idx] += 1
        places = zip(possible_places, obj_without_place)
        places.sort(key=lambda x: x[1])

        return GetBestPlaceLocationResponse(place_loc=places[0][0])

    def get_current_options(self, obj, used=None):
        if used is None:
            used = self.get_unavailable_places()
        possible_places = copy.copy(place_options[obj])
        occupied = used.intersection(set(possible_places))
        for occ in occupied:
            possible_places.remove(occ)
        return possible_places

    def get_unavailable_places(self):
        res = []
        for o in self.objects_on_plate:
            interferences = interference_map[o.place_loc]

            for i in interferences:
                res.append(i)
            res.append(o.place_loc)

        return set(res)


    def get_mapped_object(self, obj):
        for o in objects_map.keys():
            if obj in objects_map[o]:
                return o

def testFuntion(manager):

    bestLoc = GetBestPlaceLocationRequest()
    bestLoc.obj_name = 'M30_h'
    bestLoc.objects_to_pick = ['M30', 'M20', 'M30']

    res = manager.get_best_place_location(bestLoc)

    o = BackplateObject()
    o.label = bestLoc.obj_name
    o.obj_name = 'M30_h'
    o.place_type = 'ppt'
    o.place_loc = res.place_loc

    manager.add_object_to_plate(BackplateStateModifyRequest(obj=o))

    objects = manager.get_objects_from_plate(BackplateStateStatus())
    print objects

    bestLoc.obj_name = 'M20_h'
    bestLoc.objects_to_pick = ['M30', 'M20', 'M30']

    res = manager.get_best_place_location(bestLoc)

    o2 = BackplateObject()
    o2.label = bestLoc.obj_name
    o2.obj_name = 'M20_h'
    o2.place_type = 'ppt'
    o2.place_loc = res.place_loc

    manager.add_object_to_plate(BackplateStateModifyRequest(obj=o2))

    objects = manager.get_objects_from_plate(BackplateStateStatus())
    print objects

    bestLoc.obj_name = 'M30_h'
    bestLoc.objects_to_pick = ['M30', 'M20', 'M30']

    res = manager.get_best_place_location(bestLoc)

    o2 = BackplateObject()
    o2.label = bestLoc.obj_name
    o2.obj_name = 'M30_h'
    o2.place_type = 'ppt'
    o2.place_loc = res.place_loc

    manager.add_object_to_plate(BackplateStateModifyRequest(obj=o2))

    objects = manager.get_objects_from_plate(BackplateStateStatus())
    print objects

    # bestLoc.obj_name = 'F20_20_B_h'
    # bestLoc.objects_to_pick = ['F20_20_G', 'F20_20_B', 'F20_20_B', 'F20_20_G']

    # res = manager.get_best_place_location(bestLoc)

    # o2 = BackplateObject()
    # o2.label = bestLoc.obj_name
    # o2.obj_name = 'F20_20_B_h'
    # o2.place_type = 'ppt'
    # o2.place_loc = res.place_loc

    # manager.add_object_to_plate(BackplateStateModifyRequest(obj=o2))

    # objects = manager.get_objects_from_plate(BackplateStateStatus())
    # print objects

    # bestLoc.obj_name = 'F20_20_G_h'
    # bestLoc.objects_to_pick = ['F20_20_G', 'F20_20_B', 'F20_20_B', 'F20_20_G']

    # res = manager.get_best_place_location(bestLoc)

    # o2 = BackplateObject()
    # o2.label = bestLoc.obj_name
    # o2.obj_name = 'F20_20_G_h'
    # o2.place_type = 'ppt'
    # o2.place_loc = res.place_loc

    # manager.add_object_to_plate(BackplateStateModifyRequest(obj=o2))

    # objects = manager.get_objects_from_plate(BackplateStateStatus())
    # print objects




    # req = lambda: 0
    # req.obj = 'M20_v'
    # req.objects_to_pick = ['M20', 'F20_20_G', 'M20_100']



    # manager.get_place_locations(req)



    # manager.get_objects_at_location(req)
    # print manager.get_best_object(req)
    # print manager.verify_object(req)


if __name__ == '__main__':
    rospy.init_node("backplate_manager")
    manager = BackPlateManager()
    #testFuntion(manager)

    rospy.spin()
