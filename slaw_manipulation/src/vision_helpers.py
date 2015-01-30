#!/usr/bin/env python
import math
from operator import itemgetter
import cv2
import cv
import numpy as np
import itertools
from numpy.linalg import norm
# IMPORTANT pip2.7 install shapely
# install geos and glibc
# sudo apt-get install libgeos-dev
import shapely.geometry

from laser_helpers import world_to_img
import vision_constants

(LASER_POS_X, LASER_POS_Y) = world_to_img(0, 0, check=False)
MAX_INTESITY_HOLES = 150
MAX_INTESITY_RGB = 130

RGB_DETECT_GRAY = False


def pixel_to_m(pix):
    return (pix / vision_constants.PIXEL_TO_CM) / 100


def m_to_pixel(m):
    return int(m * 100 * vision_constants.PIXEL_TO_CM)


def process_rgb(rgb_img):
    frame_gray = cv2.cvtColor(rgb_img, cv.CV_RGB2GRAY)
    # gray_blurred = cv2.GaussianBlur(frame_gray, (9, 9), 0)
    gray_blurred = cv2.medianBlur(frame_gray, 9)
    # gray_blurred = cv2.bilateralFilter(frame_gray, 8, 16, 4)
    # cv2.imshow("gray_blurred", gray_blurred)


    gray_filter = cv2.adaptiveThreshold(gray_blurred,
                                        255.0,
                                        # cv.CV_ADAPTIVE_THRESH_GAUSSIAN_C,
                                        cv.CV_ADAPTIVE_THRESH_MEAN_C,
                                        cv.CV_THRESH_BINARY,
                                        9,  # neighbourhood
                                        9)
    cv2.bitwise_not(gray_filter, gray_filter)
    kernel = np.ones((5, 5), 'uint8')
    gray_dilate = cv2.dilate(gray_filter, kernel)
    size = rgb_img.shape
    size = (size[1] - 1, size[0] - 1)

    cv2.rectangle(gray_dilate, (0, 0), size,
                  0,  # color
                  20,  # thickness
                  8,  # line-type ???
                  0)  # random shit

    return gray_dilate


def filter_low_high(frame, min_val, max_val):
    low_ids = frame < min_val
    high_ids = frame > max_val

    frame[low_ids] = min_val
    frame[high_ids] = min_val
    return frame


def process_depth(depth_img):
    # # Filter NaN
    idx = np.isnan(depth_img)
    depth_img[idx] = 0

    # # Convert to UINT8 image

    depth_img = filter_low_high(depth_img, vision_constants.MIN, vision_constants.MAX)
    depth_img = depth_img / (vision_constants.MAX) * 255
    depth_img = np.uint8(depth_img)
    depth_img = cv2.medianBlur(depth_img, 3)


    # depth_img = cv2.GaussianBlur(depth_img, (9,9), 0)
    if vision_constants.FULL_VISUALIZE:
        cv2.imshow("camera_gray2", depth_img)
    # equ2 = cv2.equalizeHist(color_gray)
    #cv2.imshow("camera_gray3", equ2)


    # adaptive filter
    frame_filter = cv2.adaptiveThreshold(depth_img,
                                         255,
                                         # cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                         cv2.ADAPTIVE_THRESH_MEAN_C,
                                         cv2.THRESH_BINARY,

                                         11,  # neighbourhood
                                         2)

    # cv2.imshow("camera_filter", frame_filter)

    ## Invert Colors
    cv2.bitwise_not(frame_filter, frame_filter)


    # Dilate to merge shapes
    kernel = np.ones((7, 7), 'uint8')
    frame_dilate = cv2.dilate(frame_filter, kernel)

    size = depth_img.shape
    size = (size[1] - 1, size[0] - 1)

    # rectangle to get correctly order contours
    cv2.rectangle(frame_dilate, (0, 0), size,
                  0,  # color
                  20,  # thickness
                  8,  # line-type ???
                  0)  # random shit
    return frame_dilate


def filter_contours(contours):
    conts = []
    approx = []
    for c in contours:
        hull = None
        try:
            hull = cv2.convexHull(c)
        except Exception as e:
            print e
            continue
        area = cv2.contourArea(hull)

        if len(hull) <= 4:
            continue
        if vision_constants.MIN_AREA < area < vision_constants.MAX_AREA:

            rect = cv2.minAreaRect(c)
            box = cv2.cv.BoxPoints(rect)
            box = np.int32(box)
            vec1 = box[1] - box[2]
            vec2 = box[2] - box[3]

            axis = (np.linalg.norm(vec1), np.linalg.norm(vec2))

            # print len(hull), area, axis
            # ellipse = cv2.fitEllipse(c)
            # axis = tuple(np.int32(ellipse[1]))
            # print axis
            if (max(axis) < vision_constants.MAX_LEN) and (min(axis) > vision_constants.MIN_LEN):
                # print axis, area
                # epsilon = 0.01*cv2.arcLength(c,True)
                # a = cv2.approxPolyDP(c,epsilon,True)
                conts.append(hull)
                # conts.append(c)
                # print approx, conts[-1]
                approx.append(c)

    return conts, approx


def distance_object_line(p3, obj):
    if obj is not None:
        p1 = [LASER_POS_X - vision_constants.crop_x[0], LASER_POS_Y - vision_constants.crop_y[0]]
        p2 = obj['center']
        p3 = [p3[0] - vision_constants.crop_x[0], p3[1] - vision_constants.crop_y[0]]
        # print('x: %d y: %d ' % (obj['center'][0], obj['center'][1]))

        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]

        something = dx * dx + dy * dy

        u = ((p3[0] - p1[0]) * dx + (p3[1] - p1[1]) * dy) / float(something)

        if u > 1:
            u = 1
        elif u < 0:
            u = 0

        x = p1[0] + u * dx
        y = p1[1] + u * dy

        dx = x - p3[0]
        dy = y - p3[1]

        # Note: If the actual distance does not matter,
        # if you only want to compare what this function
        # returns to other results of this function, you
        # can just return the squared distance instead
        # (i.e. remove the sqrt) to gain a little performance

        dist = math.sqrt(dx * dx + dy * dy)

        return dist
    else:
        return 99999999999


def merge_contours(conts, dist_between=vision_constants.DIST_BETWEEN_CONTOURS):
    non_merged = False
    centers = []
    res_conts = []
    merge_conts = []
    for cnt in conts:
        try:
            hull = cv2.convexHull(cnt)
        except Exception as e:
            print e
            continue
        M = cv2.moments(hull)
        area = cv2.contourArea(hull)

        if M['m00'] == 0.0 or area > vision_constants.MAX_AREA_MERGE:
            # print area
            res_conts.append(cnt)
            continue
        centroid_x = int(M['m10'] / M['m00'])
        centroid_y = int(M['m01'] / M['m00'])

        center = np.array([centroid_x, centroid_y])
        centers.append(center)
        merge_conts.append(cnt)
    # print len(centers)
    # ret  = [x for x in centers]
    while not non_merged:
        non_merged = True
        #rem_x = None
        #rem_y = None
        idx = 0
        idy = 0
        for idx, cen1 in enumerate(centers):
            for idy, cen2 in enumerate(centers):
                # print idy, "test"
                if idy == idx:
                    continue

                dist = np.linalg.norm(cen1 - cen2)
                #print dist, pixel_to_m(dist)
                if abs(pixel_to_m(dist)) < dist_between:
                    non_merged = False

                    #print "Found one", idx, idy, dist
                    #rem_x = idx
                    #rem_y = idy
                    break
            if not non_merged:
                break
        if non_merged:
            continue
        #print 'merge', idx, idy
        merged = np.append(merge_conts[idx], merge_conts[idy], axis=0)
        # print test

        if idy > idx:
            centers.pop(idy)
            centers.pop(idx)
            merge_conts.pop(idy)
            merge_conts.pop(idx)
        else:
            centers.pop(idx)
            centers.pop(idy)
            merge_conts.pop(idy)
            merge_conts.pop(idx)

        try:
            cnt = cv2.convexHull(merged)
        except Exception as e:
            print e
            continue

        M = cv2.moments(cnt)
        if M['m00'] == 0:
            print 'merged has no area'
            continue

        merge_conts.append(cnt)

        centroid_x = int(M['m10'] / M['m00'])
        centroid_y = int(M['m01'] / M['m00'])
        center = np.array([centroid_x, centroid_y])
        centers.append(center)
    for cnt in merge_conts:
        res_conts.append(cnt)
    return res_conts


def merge_contours_holes(conts, dist_between=vision_constants.DIST_BETWEEN_CONTOURS):
    non_merged = False
    while not non_merged:
        non_merged = True
        # rem_x = None
        # rem_y = None
        idx = 0
        idy = 0
        for idx, contx in enumerate(conts):
            for p in contx:
                for idy, conty in enumerate(conts):
                    if idy == idx:
                        continue

                    #print p[0]
                    dist = cv2.pointPolygonTest(conty, tuple(p[0]), True)
                    if abs(pixel_to_m(dist)) < dist_between:
                        non_merged = False

                        #print "Found one", idx, idy, dist
                        break
                if not non_merged:
                    break
            if not non_merged:
                break
        if non_merged:
            continue
        #print 'merge', idx, idy
        merged = np.append(conts[idx], conts[idy], axis=0)
        # print test

        if idy > idx:
            conts.pop(idy)
            conts.pop(idx)
        else:
            conts.pop(idx)
            conts.pop(idy)

        try:
            cnt = cv2.convexHull(merged)
        except Exception as e:
            print e
            continue
        conts.append(cnt)
    return conts


def add_laser_feature(objects, laser_points):
    if len(objects) <= 0:
        return

    for point in laser_points:
        min_dis_index = 0
        min_dis = distance_object_line(point, objects[0])
        for index, obj in enumerate(objects):
            distance = distance_object_line(point, obj)
            if distance < min_dis:
                min_dis_index = index
                min_dis = distance
        if min_dis <= vision_constants.MIN_POINT_LASER_DIS:
            # print min_dis
            objects[min_dis_index]['laser_counter'] += 1

    for o in objects:
        if o['laser_counter'] >= vision_constants.MIN_OBJECT_LASER_COUNT:
            o['features'][-1] = True
        else:
            o['features'][-1] = False


def triangle_area(p):
    return 0.5 * norm(np.cross(p[1] - p[0], p[2] - p[0]))


def is_in_triangle(p, triangle):
    poly = shapely.geometry.Polygon(triangle)
    point = shapely.geometry.asPoint(p)
    return point.within(poly)


def triangles_intersect(t1, t2):
    p1 = shapely.geometry.Polygon(t1)
    p2 = shapely.geometry.Polygon(t2)
    try:
        inter = p1.intersection(p2)
    except:
        return False
    if inter is None:
        return False
    area = inter.area
    # print area
    if area > 0:
        return True
    else:
        return False

        # return p1.intersects(p2)

        #return p1.intersects(p2)


def add_laser_area_to_objs(objects):
    # sort by angle to center line
    objects = sorted(objects, key=itemgetter('polar_angle_abs'))
    laser_pos = np.int32([LASER_POS_X, LASER_POS_Y])
    prev_triangles = []
    for index, obj in enumerate(objects):
        # choose point of object which maximizes area
        # and doesn't intersect with prev triangles
        #
        # original bound is in crop coordinates
        box = list_crop2full_coordinates(obj['bounding_box'])
        p = itertools.permutations(range(4), 2)
        max_area = 0
        triangle_points = None
        for pair in p:
            tmp_triangle = np.array([laser_pos, box[pair[0]], box[pair[1]]])

            # tmp_area = triangle_area(tmp_triangle)
            tmp_area = abs(angle_between(tmp_triangle[1] - tmp_triangle[0], tmp_triangle[2] - tmp_triangle[0]))
            intersect = False
            for triangle in prev_triangles:
                intersect = intersect or triangles_intersect(tmp_triangle, triangle)
            if tmp_area > max_area and not intersect:
                #prev_triangles.append(tmp_triangle)
                triangle_points = tmp_triangle
                max_area = tmp_area
        if triangle_points is not None:
            prev_triangles.append(triangle_points)
            obj['laser_area'] = triangle_points
            # print ('Add laserArea %d' % index)


def add_laser_feature_v2(objects, laser_points):
    add_laser_area_to_objs(objects)
    # lp = list_crop2full_coordinates(laser_points)
    lp = laser_points
    # if enough laser scans area in the object laser_area, set laser_feature of object = True
    for obj in objects:
        obj['features'][-1] = False
        if obj['laser_area'] is not None:
            for p in lp:
                if is_in_triangle(p, obj['laser_area']):
                    obj['laser_counter'] += 1
                if obj['laser_counter'] >= vision_constants.MIN_OBJECT_LASER_COUNT:
                    obj['features'][-1] = True
                    break


def find_m20_head(cont, center, ang):
    rotated = []  # cont.copy()
    # print rotated
    # print cont#, ang
    for p in cont:
        p = rotate_point(p, center, ang)
        rotated.append(p)
    #print rotated

    rotated = np.array([[(x, y)] for (x, y) in rotated])

    leftmost = tuple(rotated[rotated[:, :, 0].argmin()][0])
    rightmost = tuple(rotated[rotated[:, :, 0].argmax()][0])
    topmost = tuple(rotated[rotated[:, :, 1].argmin()][0])
    bottommost = tuple(rotated[rotated[:, :, 1].argmax()][0])

    rotated = np.int32(rotated)
    avg_x = (leftmost[1] + rightmost[1]) / 2.
    #print avg_x
    #print bottommost[1], topmost[1]
    dist_b = abs(avg_x - bottommost[1])
    dist_t = abs(avg_x - topmost[1])
    dist_c = abs(avg_x - center[1])
    if dist_c < dist_t and dist_c < dist_b:
        dist_l = find_best_m20_match(leftmost, rotated)
        dist_r = find_best_m20_match(rightmost, rotated)
        if dist_l > dist_r:
            avg_x = leftmost[1]
        else:
            avg_x = rightmost[1]
            #print "extra check", dist_l, dist_r

    if abs(avg_x - bottommost[1]) > abs(avg_x - topmost[1]):
        return 'Down', rotated
    else:
        return 'Top', rotated


def find_best_m20_match(point, rotated_cont):
    dist = 0
    eps = 15
    for p in rotated_cont:
        if abs(p[0][1] - point[1]) > eps:
            continue
        d = np.linalg.norm(point - p[0])
        if d > dist:
            dist = d
    return dist


def rotate_point(point, center, angle):
    """Rotates a point around another center. Angle is in radian.
    Rotation is counter-clockwise"""
    # angle = math.radians(angle)
    point = point[0]
    temp_point = point[0] - center[0], point[1] - center[1]
    temp_point = (temp_point[0] * math.cos(angle) - temp_point[1] * math.sin(angle),
                  temp_point[0] * math.sin(angle) + temp_point[1] * math.cos(angle))
    temp_point = temp_point[0] + center[0], temp_point[1] + center[1]
    return temp_point


# def rotate_point(p, ang):
# x = p[0][0]
#    y = p[0][1]
#    x1 = x * math.cos(ang) - y * math.sin(ang)
#    y1 = y * math.sin(ang) + x * math.cos(ang) 
#    return np.array([int(x1), int(y1)])

def get_objects_rgb(contours, depth_img, frame_gray, global_mask=None, use_laser=False):
    # # walk through contours

    counter2 = 0
    objects = []
    for c in contours:
        mask = np.zeros(frame_gray.shape, dtype=np.uint8)
        cv2.drawContours(mask, [c], 0, 255, -1)

        #make contours a bit smaller    
        kernel = np.ones((3, 3), 'uint8')
        mask_erode = cv2.erode(mask, kernel)
        mean_val = cv2.mean(frame_gray, mask=mask_erode)
        intensity = mean_val[0]

        #redect dark shapes
        if intensity > MAX_INTESITY_RGB:
            if not RGB_DETECT_GRAY:
                continue
        else:
            ## Get smaller frame to detect a second time
            leftmost = tuple(c[c[:, :, 0].argmin()][0])
            rightmost = tuple(c[c[:, :, 0].argmax()][0])
            topmost = tuple(c[c[:, :, 1].argmin()][0])
            bottommost = tuple(c[c[:, :, 1].argmax()][0])
            eps = 5
            #print leftmost, rightmost, topmost, bottommost
            # get object from gray image with smaller mask
            object_image = frame_gray[topmost[1] - eps:bottommost[1] + eps, leftmost[0] - eps:rightmost[0] + eps].copy()
            #blur = object_image

            ## Some blur and OTSU thresholding
            blur = cv2.GaussianBlur(object_image, (3, 3), 0)
            ret, threshold = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            cv2.bitwise_not(threshold, threshold)
            size = object_image.shape
            size = (size[1] - 1, size[0] - 1)

            ##Find second contour in smaller image
            conts, _ = cv2.findContours(threshold.copy(),
                                        cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
            ### If more than one contour take largest
            idx = 0
            if not len(conts) == 1:
                min_area = 0
                for i, cont in enumerate(conts):
                    try:
                        h = cv2.convexHull(cont)
                    except Exception as e:
                        print e
                        continue
                    area_tmp = cv2.contourArea(h)
                    if area_tmp > min_area:
                        min_area = area_tmp
                        idx = i

                        #if vision_constants.FULL_VISUALIZE:
                        #    cv2.imshow("object_false", threshold)
                        #print counter

            ### Crop to full image
            topleft = np.array([leftmost[0] - eps, topmost[1] - eps])
            if vision_constants.FULL_VISUALIZE:
                cv2.drawContours(blur, [conts[idx]], -1, (255, 255, 0), 2)
                string = 'object' + str(counter2)
                counter2 += 1
                cv2.imshow(string, blur)
            #cv.WaitKey(0)

            for p in conts[idx]:
                p += topleft

            c = conts[idx]

        obj = None
        try:
            obj = get_features(c, depth_img, frame_gray, use_laser)
        except Exception as e:
            print e
            continue

        #cv2.drawContours(mask, [c], 0, 255, -1)
        if global_mask is not None:
            cv2.drawContours(global_mask, [c], 0, 255, -1)

        #cv2.drawContours(blur, [a], -1, (255,255,0), 2)
        #string = 'object' + str(counter2)
        #counter2+=1
        #cv2.imshow(string, blur)
        #cv.WaitKey(0)

        objects.append(obj)
    return objects


def get_objects(contours, depth_img, frame_gray, global_mask=None, use_laser=False):
    # # walk through contours
    objects = []
    for c in contours:
        obj = None
        try:
            obj = get_features(c, depth_img, frame_gray, use_laser)
        except Exception as e:
            print e
            continue
        epsilon = 0.01 * cv2.arcLength(c, True)
        obj['cont'] = cv2.approxPolyDP(c, epsilon, True)
        objects.append(obj)
        if global_mask is not None:
            cv2.drawContours(global_mask, [c], 0, 255, -1)
    return objects


def get_features(c, depth_img, frame_gray, use_laser):
    obj = {}
    epsilon = 0.01 * cv2.arcLength(c, True)
    a = cv2.approxPolyDP(c, epsilon, True)
    defects_count = 0
    defects = None
    try:
        hull = cv2.convexHull(a, returnPoints=False)
        if len(hull) > 3 and len(a) > 3:
            defects = cv2.convexityDefects(a, hull)
    except Exception as e:
        print e
        defects = None
    if defects is not None:
        defects_count = defects.shape[0]

    rect = cv2.minAreaRect(c)
    box = cv2.cv.BoxPoints(rect)
    box = np.int32(box)

    vec1 = box[1] - box[2]
    vec2 = box[2] - box[3]

    center = box[0] - (vec1 - vec2) / 2
    obj['center'] = center

    axis2 = (np.linalg.norm(vec1), np.linalg.norm(vec2))

    aspect_ratio = max(axis2) / min(axis2)

    mask = np.zeros(depth_img.shape, dtype=np.uint8)

    cv2.drawContours(mask, [c], 0, 255, -1)

    kernel = np.ones((9, 9), 'uint8')
    mask_erode = cv2.erode(mask, kernel)

    mask_dilude = cv2.dilate(mask, kernel)
    border_mask = mask_dilude - mask
    # get object from gray image with smaller mask
    # object_image = color_img.copy()
    #object_image[border_mask == 0] = 0
    #cv2.imshow("object", object_image)

    mean_val = cv2.mean(frame_gray, mask=mask_erode)
    intensity = mean_val[0]

    mask_center = cv2.erode(mask_erode, kernel)
    border_mask_int = mask_erode - mask_center

    mean_center = cv2.mean(frame_gray, mask=mask_center)

    mean_around = cv2.mean(frame_gray, mask=border_mask_int)

    intensity_dif = mean_around[0] - mean_center[0]
    #mean_rgb = cv2.mean(color_img, mask = mask_erode)
    #mean_rgb = mean_rgb[0:-1]



    idx = np.isnan(depth_img)
    border_mask[idx] = 0
    mask_erode[idx] = 0

    idx = depth_img <= vision_constants.MIN
    border_mask[idx] = 0
    mask_erode[idx] = 0

    #print border
    mean_depth_around = cv2.mean(depth_img, mask=border_mask)
    mean_depth = cv2.mean(depth_img, mask=mask_erode)
    min_depth, max_depth, min_loc, max_loc = cv2.minMaxLoc(depth_img, mask=mask_erode)  #min, max, min_loc, max_loc

    obj['features'] = [cv2.contourArea(cv2.convexHull(c)), min(axis2), max(axis2), aspect_ratio, intensity,
                       intensity_dif,
                       defects_count, mean_depth_around[0] - mean_depth[0],
                       mean_depth_around[0] - min_depth, len(np.nonzero(mask_erode)[0]),
                       False]  #add feature for laser

    if np.linalg.norm(vec1) < np.linalg.norm(vec2):
        obj['angle'] = np.arctan2(vec1[1], vec1[0])
    else:
        obj['angle'] = np.arctan2(vec2[1], vec2[0])

    obj['bounding_box'] = box
    obj['contour'] = c
    if use_laser:
        obj['polar_angle_abs'] = abs(get_laser_polar_angle(single_crop2full_coordinates(center)))
        #decribes the area(triangle) of two object coordinates and the laser position (laser coverage)
        obj['laser_area'] = None
        #to count how much laser point are close to the object
        obj['laser_counter'] = 0

    return obj


def get_holes(contours, frame_gray, global_mask=None):
    # # walk through contours
    holes = []
    for c in contours:
        mask = np.zeros(frame_gray.shape, dtype=np.uint8)

        cv2.drawContours(mask, [c], 0, 255, -1)
        if global_mask is not None:
            cv2.drawContours(global_mask, [c], 0, 255, -1)

        #make contours a bit smaller    
        kernel = np.ones((3, 3), 'uint8')
        mask_erode = cv2.erode(mask, kernel)
        mean_val = cv2.mean(frame_gray, mask=mask_erode)
        intensity = mean_val[0]

        #skip to bright holes
        if intensity > MAX_INTESITY_HOLES:
            continue

        ## Get smaller frame to detect a second time
        leftmost = tuple(c[c[:, :, 0].argmin()][0])
        rightmost = tuple(c[c[:, :, 0].argmax()][0])
        topmost = tuple(c[c[:, :, 1].argmin()][0])
        bottommost = tuple(c[c[:, :, 1].argmax()][0])
        eps = 10
        #print leftmost, rightmost, topmost, bottommost
        # get object from gray image with smaller mask
        object_image = frame_gray[topmost[1] - eps:bottommost[1] + eps, leftmost[0] - eps:rightmost[0] + eps].copy()
        #blur = object_image

        ## Some blur and OTSU thresholding
        blur = cv2.GaussianBlur(object_image, (3, 3), 0)
        ret, threshold = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        cv2.bitwise_not(threshold, threshold)
        size = object_image.shape
        size = (size[1] - 1, size[0] - 1)

        ##Find second contour in smaller image
        conts, _ = cv2.findContours(threshold.copy(),
                                    cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
        ### If more than one contour probably not an hole..
        if not len(conts) == 1:
            if vision_constants.FULL_VISUALIZE:
                cv2.imshow("object_false", threshold)
            #print counter
            continue

        ## Approximate contour
        c = conts[0]
        epsilon = 0.01 * cv2.arcLength(c, True)
        a = cv2.approxPolyDP(c, epsilon, True)
        try:
            hull = cv2.convexHull(a, returnPoints=False)
            defects = cv2.convexityDefects(a, hull)
        except Exception as e:
            print e
            defects = None
        defects_count = 0

        if defects is not None:
            defects_count = defects.shape[0]


        #cv2.drawContours(blur, [a], -1, (255,255,0), 2)
        #string = 'object' + str(counter2)
        #counter2+=1
        #cv2.imshow(string, blur)
        #cv.WaitKey(0)


        ### Crop to full image
        topleft = np.array([leftmost[0] - eps, topmost[1] - eps])

        for p in conts[0]:
            p += topleft

        hole = {}
        c = conts[0]
        #ellipse = cv2.fitEllipse(c)
        # rect = cv2.minAreaRect(c)
        #center = np.int32(ellipse[0])

        rect = cv2.minAreaRect(c)
        box = cv2.cv.BoxPoints(rect)
        box = np.int32(box)
        #center = tuple(np.int32(ellipse[0]))
        vec1 = box[1] - box[2]
        vec2 = box[2] - box[3]
        #hole['center'] = center
        hole['center'] = box[0] - (vec1 - vec2) / 2

        axis2 = (np.linalg.norm(vec1), np.linalg.norm(vec2))

        aspect_ratio = max(axis2) / min(axis2)

        hole['features'] = [cv2.contourArea(c), min(axis2), max(axis2), aspect_ratio, len(a), defects_count]

        hole['contour'] = conts[0]
        if global_mask is not None:
            cv2.drawContours(global_mask, [conts[0]], 0, 255, -1)

        if np.linalg.norm(vec1) < np.linalg.norm(vec2):
            hole['angle'] = np.arctan2(vec1[1], vec1[0])
        else:
            hole['angle'] = np.arctan2(vec2[1], vec2[0])

        holes.append(hole)
    return holes


def single_crop2full_coordinates(point):
    return [point[0] + vision_constants.crop_x[0], point[1] + vision_constants.crop_y[0]]


def list_crop2full_coordinates(point_list):
    points = []
    for point in point_list:
        points.append(single_crop2full_coordinates(point))
    return points


def single_full2crop_coordinates(point):
    return [point[0] - vision_constants.crop_x[0], point[1] - vision_constants.crop_y[0]]


def list_full2crop_coordinates(point_or_points):
    points = []
    for point in point_or_points:
        points.append(single_full2crop_coordinates(point))
        points = np.int32(points)
    return points


def get_laser_polar_angle(point):
    laser_axis_direction = np.array([LASER_POS_X, 1])
    point_vector = point - np.array([LASER_POS_X, LASER_POS_Y])
    return angle_between(laser_axis_direction, point_vector)


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    angle = np.arccos(np.dot(v1_u, v2_u))
    if np.isnan(angle):
        if (v1_u == v2_u).all():
            return 0.0
        else:
            return np.pi
    return angle


def get_darkest_hole(objects, img_gray, size):
    best = 100000
    obj = None
    #cross_w = size[0] / 2 + vision_constants.CROSS_OFF_X
    #cross_h = size[1] / 2 + vision_constants.CROSS_OFF_Y

    #cross = np.int32([cross_w, cross_h])
    #min_intensity = 1000000 
    for o in objects:
        mask = np.zeros(img_gray.shape, dtype=np.uint8)
        c = o['contour']
        cv2.drawContours(mask, [c], 0, 255, -1)

        #kernel = np.ones((3, 3), 'uint8')
        #mask_erode = cv2.erode(mask, kernel)


        # get object from gray image with smaller mask
        object_image = img_gray.copy()
        object_image[mask == 0] = 0
        cv2.imshow("object_test", object_image)

        mean_val = cv2.mean(img_gray, mask=mask)
        #mean_rgb = cv2.mean(color_img, mask = mask_erode)
        #mean_rgb = mean_rgb[0:-1]
        intensity = mean_val[0]

        #print intensity
        #if intensity > 30 and best < 100000:
        #    continue

        #center = o['center']
        #dist = np.linalg.norm(center - cross)
        if intensity < best:  # or (obj['center'][0] > cross_w and o['center'][0] < cross_w and intensity < 80) :
            #if obj is not None and o['center'][0] > cross_w and obj['center'][0] < cross_w and best < 80:
            #     continue
            best = intensity
            obj = o
    return obj


def get_closest(objects, size, training=False):
    best = 100000
    obj = None
    if training:
        cross_w = size[0] / 2  # + vision_constants.CROSS_OFF_X
        cross_h = size[1] / 2  # + vision_constants.CROSS_OFF_Y
    else:
        cross_w = size[0] / 2 + vision_constants.CROSS_OFF_X
        cross_h = size[1] / 2 + vision_constants.CROSS_OFF_Y

    cross = np.int32([cross_w, cross_h])

    for o in objects:
        center = o['center']
        dist = np.linalg.norm(center - cross)

        if dist < best:
            best = dist
            obj = o
    return obj

