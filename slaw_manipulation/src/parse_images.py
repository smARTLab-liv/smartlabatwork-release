#!/usr/bin/env python
import traceback
import cv
import cv2
import sys, getopt
import os
import numpy as np
from detect_objects_depth import DetectObjectsDepth
from detect_objects_rgb import DetectObjectsRGB
from detect_holes_rgb import DetectHolesRGB


detect_depth = DetectObjectsDepth(training=True)
detect_rgb = DetectObjectsRGB(training=True)
detect_holes_rgb = DetectHolesRGB(training=True)

OBJECTS = ["F20_20_B",  # 1
           "F20_20_G",  # 2
           "S40_40_B",  # 3
           "S40_40_G",  # 4
           "M20_100",  # 5
           "M20",  # 6
           "M30",  # 7
           "R20",  # 8
           "V20",  # 9
]

LOW_OBJECTS = ["M20_vertical", "M20_horizontal", "M30_horizontal", "F20_20_B_horizontal", "F20_20_G_horizontal",
               "R20_horizontal", "V20_horizontal", "M20_100_horizontal"]

features = ['area', 'min_axis', 'max_axis', 'aspect_ratio', 'intensity', 'intensity_dif', 'convex_defects',
            'rel_mean_depth', 'rel_max_depth', 'valid_depth_readings', 'above_laser']

features_holes = ['area', 'min_axis', 'max_axis', 'aspect_ratio', 'corners', 'convex_defects']


def print_weka_header(folder, orientation, holes):
    print "@RELATION detection"
    if holes:
        for f in features_holes:
            print "@ATTRIBUTE " + f + " NUMERIC"
    else:
        for f in features:
            print "@ATTRIBUTE " + f + " NUMERIC"

    objects_str = ''
    for o in orientation:
        or_folder = os.path.join(folder, o)
        object_folders = [d for d in os.listdir(or_folder) if not os.path.isfile(os.path.join(or_folder, d))]

        for obj in object_folders:
            objects_str += "_".join([obj, o]) + ', '

    print "@ATTRIBUTE class {" + objects_str[:-2] + "}"
    print "@DATA"


def arg_error(e=''):
    print (str(e))
    print "-o --orientation    v, h, vh, hv (horizontal, vertical)"
    print "-d --detect   r, d, rd, dr (depth, rgb)"
    print "-w --wait  wait for key after each image)"
    print "-s --silent no waits no errors"
    print ""
    sys.exit(2)


if __name__ == "__main__":
    crop_y = [180, 420]
    crop_x = [200, 480]

    # import rospy
    # rospy.init_node('test')
    # detect_holes_rgb = DetectHolesRGB(training = False)
    #detect_depth = DetectObjectsDepth(training = False)
    #detect_rgb = DetectObjectsRGB(training = False)

    folder = "./trainingSet/"
    mask = None
    wait = 5

    try:
        opts, args = getopt.getopt(sys.argv[1:], 'pso:d:wh',
                                   ['holes', 'silent', 'orientation=', 'detect=', 'wait', '--help'])
    except getopt.GetoptError as e:
        arg_error(e)

    orientation = []
    detect_via = []
    silent = False
    holes = False
    for o, a in opts:
        if o in ('-o', '--orientation'):
            if 'v' in a:
                orientation.append('vertical')
            if 'h' in a:
                orientation.append('horizontal')
        if o in ('-d', '--detect'):
            if 'r' in a:
                detect_via.append('rgb')
            if 'd' in a:
                detect_via.append('depth')
        if o in ('-w', '--wait'):
            wait = 0
        if o in ('-s', '--silent'):
            wait = 5
            silent = True
        if o in ('-p', '--holes'):
            holes = True
            crop_y = [180, 420]
            crop_x = [160, 520]

        if o in ('-h', '--help'):
            arg_error()
            exit(2)

    if len(orientation) == 0:
        orientation = ['horizontal', 'vertical']
    if len(detect_via) == 0:
        detect_via = ['depth', 'rgb']

    if not silent:
        print orientation, detect_via

    print_weka_header(folder, orientation, holes)

    for o in orientation:
        or_folder = os.path.join(folder, o)
        object_folders = [d for d in os.listdir(or_folder) if not os.path.isfile(os.path.join(or_folder, d))]

        for obj in object_folders:
            #print obj
            depth_folder = os.path.join(or_folder, obj, 'depth')
            color_folder = os.path.join(or_folder, obj, 'rgb')
            #print "reading color images from: %s and depth from %s"%(color_folder, depth_folder)
            color_imgs = [f for f in os.listdir(color_folder) if os.path.isfile(os.path.join(color_folder, f))]
            depth_imgs = [f for f in os.listdir(depth_folder) if os.path.isfile(os.path.join(depth_folder, f))]
            color_imgs.sort()
            depth_imgs.sort()

            for rgb, depth in zip(color_imgs, depth_imgs):
                rgb_f = os.path.join(color_folder, rgb)
                depth_f = os.path.join(depth_folder, depth)

                #print rgb_f, depth_f
                data_rgb = np.load(rgb_f)
                frame_rgb = data_rgb["arr_0"]
                data_rgb.close()

                data_depth = np.load(depth_f)

                frame_depth = data_depth["arr_0"]
                data_depth.close()
                #frame_rgb = np.array(frame_rgb, dtype=np.uint8)
                frame_rgb = frame_rgb[crop_y[0]:crop_y[1], crop_x[0]:crop_x[1]]
                frame_depth = frame_depth[crop_y[0]:crop_y[1], crop_x[0]:crop_x[1]]

                cv2.imshow("rgb", frame_rgb)
                cv2.imshow("depth", frame_depth)

                #feature_vector = detect.process_images(frame_depth, frame_rgb)

                try:
                    if not holes:
                        label = "_".join([obj, o])
                        high = '1'
                        if label in LOW_OBJECTS:
                            high = '0'
                        if 'depth' in detect_via:
                            features = detect_depth.process_images(frame_depth, frame_rgb)
                            print ",".join(str(x) for x in features[:-1]) + "," + high + ', ' + label

                        if 'rgb' in detect_via:
                            features = detect_rgb.process_images(frame_depth, frame_rgb)
                            print ",".join(str(x) for x in features[:-1]) + "," + high + ', ' + label
                    else:
                        features = detect_holes_rgb.process_image(frame_rgb)
                        print ",".join(str(x) for x in features) + "," + "_".join([obj, o])
                except Exception as e:
                    if not silent:
                        print "could not read %s" % rgb, e
                        traceback.print_stack()
                        cv2.imshow("rgb", frame_rgb)
                        cv2.imshow("depth", frame_depth)
                        cv.WaitKey(0)
                        continue
                cv.WaitKey(wait)


    
