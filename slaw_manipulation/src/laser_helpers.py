#!/usr/bin/env python
import numpy as np
from math import sin, cos
import math
from vision_constants import PIXEL_TO_CM
import rospy
from slaw_srvs.srv import SetSide, SetSideResponse


MAX_Y = 0.65
MIN_Y = 0.05
MIN_X = -0.4
MAX_X = 0.4

SIZE_X = 640
SIZE_Y = 480

OFFSET_ROTATION = math.pi / 4 + 0.05
OFFSET_X = -10  # -16
OFFSET_Y = -110  # -95

SIDE_OFFSET_ROTATION = math.pi / 4 + 0.05
SIDE_OFFSET_X = -10  # -16
SIDE_OFFSET_Y = -110  # -95

FRONT_OFFSET_ROTATION = 3. * math.pi / 4. + 0.05
FRONT_OFFSET_X = 30  # -16
FRONT_OFFSET_Y = -110  # -95


def set_offsets_to(req):
    global OFFSET_X, OFFSET_Y, OFFSET_ROTATION
    if req.side == 'side':
        OFFSET_X = SIDE_OFFSET_X
        OFFSET_Y = SIDE_OFFSET_Y
        OFFSET_ROTATION = SIDE_OFFSET_ROTATION
    elif req.side == 'front':
        OFFSET_X = FRONT_OFFSET_X
        OFFSET_Y = FRONT_OFFSET_Y
        OFFSET_ROTATION = FRONT_OFFSET_ROTATION

   # print req.side

    rospy.set_param('platform_direction', req.side)

    return SetSideResponse()



def create_laser_img(laser_msg):
    laser_image = np.zeros((SIZE_Y, SIZE_X), dtype=np.uint8)
    min_angle = laser_msg.angle_min
    angle_increment = laser_msg.angle_increment

    angle = min_angle - angle_increment

    points = []

    # print res

    for r in laser_msg.ranges:
        angle += angle_increment
        #print r
        if np.isnan(r) or np.isinf(r):
            continue
        res = laser_range_to_xy(angle, r)
        if res is None:
            continue
        (x, y) = res
        #print x, y
        points.append(res)
        laser_image[int(y), int(x)] = 255
    return laser_image, points


def laser_range_to_xy(ang, ran):
    x = ran * cos(ang - OFFSET_ROTATION)
    y = -ran * sin(ang - OFFSET_ROTATION)
    return world_to_img(x, y)


def set_offset_x(val):
    global OFFSET_X
    OFFSET_X = val


def set_offset_y(val):
    global OFFSET_Y
    OFFSET_Y = val


def checkBounds(x, y):
    if x < MIN_X or x > MAX_X:
        return False
    if y < MIN_Y or y > MAX_Y:
        return False
    return True


def world_to_img(x, y, check=True):
    if check and not checkBounds(x, y):
        return None
    x = (x - MIN_X) / (MAX_X - MIN_X) * SIZE_X - OFFSET_X
    y = (y - MIN_Y) / (MAX_Y - MIN_Y) * SIZE_Y - OFFSET_Y
    scale = (MAX_Y - MIN_Y) * 100. / SIZE_Y * PIXEL_TO_CM
    x *= scale
    y *= scale
    if x < 0 or x > SIZE_X:
        return None
    if y < 0 or y > SIZE_Y:
        return None
    return int(SIZE_X - x), int(y)
