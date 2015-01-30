#!/usr/bin/env python

def j48_depth_horizontal(features):
    (area, min_axis, max_axis, aspect_ratio, intensity, intensity_dif, convex_defects, rel_mean_depth, rel_max_depth,
     valid_depth_readings, laser) = features
    if max_axis <= 50.328918:
        if min_axis <= 39.051248:
            if max_axis <= 33:
                return "M20_h"
            if max_axis > 33:
                return "RV20_h"
        if min_axis > 39.051248:
            return "M30_h"
    if max_axis > 50.328918:
        if intensity <= 108.597713:
            if area <= 2657:
                if max_axis <= 92.800862:
                    return "F20_20_B_h"
                if max_axis > 92.800862:
                    return "M20_100_h"
            if area > 2657:
                return "S40_40_B_h"
        if intensity > 108.597713:
            if aspect_ratio <= 2.360427:
                return "S40_40_G_h"
            if aspect_ratio > 2.360427:
                return "F20_20_G_h"


def j48_rgb_horizontal(features):
    (area, min_axis, max_axis, aspect_ratio, intensity, intensity_dif, convex_defects, rel_mean_depth, rel_max_depth,
     valid_depth_readings, laser) = features
    if intensity <= 70.649509:
        if max_axis <= 77.897368:
            return "RV20_h"
        if max_axis > 77.897368:
            if min_axis <= 25.806976:
                return "F20_20_B_h"
            if min_axis > 25.806976:
                if min_axis <= 32.015621:
                    return "M20_100_h"
                if min_axis > 32.015621:
                    return "S40_40_B_h"
    if intensity > 70.649509:
        if aspect_ratio <= 1.487734:
            if min_axis <= 31.32092:
                return "M20_h"
            if min_axis > 31.32092:
                return "M30_h"
        if aspect_ratio > 1.487734:
            if rel_max_depth <= 0.025494:
                if valid_depth_readings <= 49:
                    if max_axis <= 26.627054:
                        return "RV20_h"
                    if max_axis > 26.627054:
                        return "M30_h"
                if valid_depth_readings > 49:
                    return "F20_20_G_h"
            if rel_max_depth > 0.025494:
                return "S40_40_G_h"



def j48_holes(features):
    (area, min_axis, max_axis, aspect_ratio, corners, convex_defects) = features
    if min_axis <= 24.020824:
        if max_axis <= 37.013511:
            if max_axis <= 19.79899:
                return "F20_20_vertical"
            if max_axis > 19.79899:
                return "M20_vertical"
        if max_axis > 37.013511:
            if min_axis <= 20.024984:
                return "F20_20_horizontal"
            if min_axis > 20.024984:
                return "M30_vertical"
    if min_axis > 24.020824:
        if min_axis <= 28.071338:
            if max_axis <= 33:
                if max_axis <= 27:
                    return "RV20_vertical"
                if max_axis > 27:
                    return "M20_horizontal"
            if max_axis > 33:
                return "RV20_horizontal"
        if min_axis > 28.071338:
            if max_axis <= 45.01111:
                if convex_defects <= 3:
                    return "M30_horizontal"
                if convex_defects > 3:
                    return "S40_40_vertical"
            if max_axis > 45.01111:
                if corners <= 6:
                    return "S40_40_horizontal"
                if corners > 6:
                    return "M20_100_horizontal"


def j48_rgb(features):
    (area, min_axis, max_axis, aspect_ratio, intensity, intensity_dif, convex_defects, rel_mean_depth, rel_max_depth,
     valid_depth_readings, laser) = features
    if max_axis <= 69.354164:
        if valid_depth_readings <= 239:
            if intensity_dif <= -8.790419:
                if area <= 530:
                    return "M20_h"
                if area > 530:
                    if max_axis <= 34.785054:
                        return "RV20_v"
                    if max_axis > 34.785054:
                        return "F20_20_v"
            if intensity_dif > -8.790419:
                if min_axis <= 26.476405:
                    if min_axis <= 19.416488:
                        if intensity <= 130.748503:
                            if convex_defects <= 3:
                                return "M20_v"
                            if convex_defects > 3:
                                return "F20_20_v"
                        if intensity > 130.748503:
                            if rel_max_depth <= 0.001579:
                                return "M30_h"
                            if rel_max_depth > 0.001579:
                                return "F20_20_G_h"
                    if min_axis > 19.416488:
                        if intensity <= 33.639723:
                            if area <= 465.5:
                                return "F20_20_v"
                            if area > 465.5:
                                return "M20_100_HD_v"
                        if intensity > 33.639723:
                            return "F20_20_v"
                if min_axis > 26.476405:
                    if intensity <= 62.380952:
                        return "M20_100_HU_v"
                    if intensity > 62.380952:
                        if min_axis <= 35.468296:
                            return "F20_20_v"
                        if min_axis > 35.468296:
                            return "S40_40_v"
        if valid_depth_readings > 239:
            if rel_mean_depth <= 0.017937:
                if valid_depth_readings <= 609:
                    if min_axis <= 29.068884:
                        return "RV20_v"
                    if min_axis > 29.068884:
                        return "S40_40_v"
                if valid_depth_readings > 609:
                    return "M30_h"
            if rel_mean_depth > 0.017937:
                if rel_mean_depth <= 0.031211:
                    return "RV20_h"
                if rel_mean_depth > 0.031211:
                    return "M30_v"
    if max_axis > 69.354164:
        if intensity <= 83.123164:
            if area <= 2274.5:
                if min_axis <= 25.806976:
                    return "F20_20_B_h"
                if min_axis > 25.806976:
                    return "M20_100_h"
            if area > 2274.5:
                return "S40_40_B_h"
        if intensity > 83.123164:
            if rel_max_depth <= 0.025495:
                return "F20_20_G_h"
            if rel_max_depth > 0.025495:
                return "S40_40_G_h"


def j48_depth(features):
    (area, min_axis, max_axis, aspect_ratio, intensity, intensity_dif, convex_defects, rel_mean_depth, rel_max_depth,
     valid_depth_readings, laser) = features
    if area <= 1474.5:
        if valid_depth_readings <= 408:
            if area <= 1088:
                if intensity <= 88.538095:
                    if area <= 608:
                        if valid_depth_readings <= 139:
                            if rel_mean_depth <= 0.678773:
                                if max_axis <= 30:
                                    if area <= 553:
                                        return "RV20_v"
                                    if area > 553:
                                        return "M20_h"
                                if max_axis > 30:
                                    return "RV20_v"
                            if rel_mean_depth > 0.678773:
                                return "M20_v"
                        if valid_depth_readings > 139:
                            return "M20_v"
                    if area > 608:
                        if intensity_dif <= -11.417989:
                            if area <= 745:
                                if valid_depth_readings <= 181:
                                    return "M20_h"
                                if valid_depth_readings > 181:
                                    return "RV20_v"
                            if area > 745:
                                return "RV20_v"
                        if intensity_dif > -11.417989:
                            if area <= 738.5:
                                if intensity_dif <= 19.461123:
                                    return "M20_100_HD_v"
                                if intensity_dif > 19.461123:
                                    return "RV20_v"
                            if area > 738.5:
                                if intensity <= 53.856916:
                                    return "M20_100_HD_v"
                                if intensity > 53.856916:
                                    return "RV20_v"
                if intensity > 88.538095:
                    if valid_depth_readings <= 78:
                        if intensity_dif <= 50.248952:
                            return "F20_20_v"
                        if intensity_dif > 50.248952:
                            return "RV20_v"
                    if valid_depth_readings > 78:
                        if area <= 788:
                            if intensity_dif <= -83.123111:
                                return "M20_h"
                            if intensity_dif > -83.123111:
                                return "RV20_v"
                        if area > 788:
                            return "RV20_v"
            if area > 1088:
                if valid_depth_readings <= 4:
                    return "M20_100_HU_v"
                if valid_depth_readings > 4:
                    return "M20_100_HD_v"
        if valid_depth_readings > 408:
            if rel_mean_depth <= 0.035095:
                return "RV20_h"
            if rel_mean_depth > 0.035095:
                return "M30_v"
    if area > 1474.5:
        if max_axis <= 72.835431:
            if valid_depth_readings <= 240:
                return "S40_40_v"
            if valid_depth_readings > 240:
                return "M30_h"
        if max_axis > 72.835431:
            if intensity <= 108.716075:
                if area <= 2643.5:
                    if max_axis <= 92.800862:
                        return "F20_20_B_h"
                    if max_axis > 92.800862:
                        return "M20_100_h"
                if area > 2643.5:
                    return "S40_40_B_h"
            if intensity > 108.716075:
                if aspect_ratio <= 2.360427:
                    return "S40_40_G_h"
                if aspect_ratio > 2.360427:
                    return "F20_20_G_h"


def j48_depth_with_laser(features):
    (area, min_axis, max_axis, aspect_ratio, intensity, intensity_dif, convex_defects, rel_mean_depth, rel_max_depth,
     valid_depth_readings, above_laser) = features
    if above_laser <= 0:
        if max_axis <= 66.241981:
            if max_axis <= 37.656341:
                if area <= 633:
                    return "M20_v"
                if area > 633:
                    return "M20_h"
            if max_axis > 37.656341:
                if min_axis <= 39.051248:
                    return "RV20_h"
                if min_axis > 39.051248:
                    return "M30_h"
        if max_axis > 66.241981:
            if intensity <= 108.716075:
                if max_axis <= 92.800862:
                    return "F20_20_B_h"
                if max_axis > 92.800862:
                    return "M20_100_h"
            if intensity > 108.716075:
                return "F20_20_G_h"
    if above_laser > 0:
        if area <= 1757:
            if valid_depth_readings <= 408:
                if intensity <= 88.538095:
                    if area <= 1118:
                        if intensity_dif <= -11.417989:
                            return "RV20_v"
                        if intensity_dif > -11.417989:
                            if intensity_dif <= 18.873563:
                                return "M20_100_HD_v"
                            if intensity_dif > 18.873563:
                                if max_axis <= 35.468296:
                                    return "RV20_v"
                                if max_axis > 35.468296:
                                    return "M20_100_HD_v"
                    if area > 1118:
                        if valid_depth_readings <= 6:
                            return "M20_100_HU_v"
                        if valid_depth_readings > 6:
                            return "M20_100_HD_v"
                if intensity > 88.538095:
                    if valid_depth_readings <= 78:
                        return "F20_20_v"
                    if valid_depth_readings > 78:
                        return "RV20_v"
            if valid_depth_readings > 408:
                return "M30_v"
        if area > 1757:
            if max_axis <= 72.835431:
                return "S40_40_v"
            if max_axis > 72.835431:
                if intensity <= 110.662602:
                    return "S40_40_B_h"
                if intensity > 110.662602:
                    return "S40_40_G_h"


def j48_with_laser_rgb(features):
    (area, min_axis, max_axis, aspect_ratio, intensity, intensity_dif, convex_defects, rel_mean_depth, rel_max_depth,
     valid_depth_readings, above_laser) = features
    if above_laser <= 0:
        if max_axis <= 58.189346:
            if intensity_dif <= -17.524671:
                if min_axis <= 31.32092:
                    return "M20_h"
                if min_axis > 31.32092:
                    return "M30_h"
            if intensity_dif > -17.524671:
                if valid_depth_readings <= 269:
                    if intensity <= 88.211149:
                        return "M20_v"
                    if intensity > 88.211149:
                        return "F20_20_G_h"
                if valid_depth_readings > 269:
                    return "RV20_h"
        if max_axis > 58.189346:
            if intensity <= 55.44186:
                if min_axis <= 25.806976:
                    return "F20_20_B_h"
                if min_axis > 25.806976:
                    return "M20_100_h"
            if intensity > 55.44186:
                return "F20_20_G_h"
    if above_laser > 0:
        if area <= 1150:
            if valid_depth_readings <= 262:
                if intensity_dif <= -8.735467:
                    if max_axis <= 34.785054:
                        return "RV20_v"
                    if max_axis > 34.785054:
                        return "F20_20_v"
                if intensity_dif > -8.735467:
                    if intensity_dif <= 18.88919:
                        if min_axis <= 26.476405:
                            if intensity <= 38.796748:
                                if intensity <= 16.997868:
                                    return "RV20_v"
                                if intensity > 16.997868:
                                    return "M20_100_HD_v"
                            if intensity > 38.796748:
                                return "F20_20_v"
                        if min_axis > 26.476405:
                            return "M20_100_HU_v"
                    if intensity_dif > 18.88919:
                        if min_axis <= 15.652476:
                            return "S40_40_G_h"
                        if min_axis > 15.652476:
                            return "F20_20_v"
            if valid_depth_readings > 262:
                if rel_max_depth <= 0.034922:
                    if intensity <= 108.754579:
                        return "RV20_v"
                    if intensity > 108.754579:
                        return "S40_40_v"
                if rel_max_depth > 0.034922:
                    return "M30_v"
        if area > 1150:
            if intensity <= 70.977505:
                return "S40_40_B_h"
            if intensity > 70.977505:
                if max_axis <= 76.400262:
                    return "S40_40_v"
                if max_axis > 76.400262:
                    return "S40_40_G_h"


def j48_july_depth(features):
    (area, min_axis, max_axis, aspect_ratio, intensity, intensity_dif, convex_defects, rel_mean_depth, rel_max_depth,
     valid_depth_readings, above_laser) = features
    if above_laser <= 0:
        if max_axis <= 66.241981:
            if max_axis <= 37.656341:
                if area <= 633:
                    if rel_mean_depth <= 0.014687:
                        if convex_defects <= 1:
                            if area <= 593.5:
                                return "M20_vertical"
                            if area > 593.5:
                                return "M20_horizontal"
                        if convex_defects > 1:
                            return "M20_horizontal"
                    if rel_mean_depth > 0.014687:
                        if rel_mean_depth <= 0.070144:
                            return "M20_vertical"
                        if rel_mean_depth > 0.070144:
                            if intensity_dif <= 54.992901:
                                return "M20_vertical"
                            if intensity_dif > 54.992901:
                                return "M20_horizontal"
                if area > 633:
                    return "M20_horizontal"
            if max_axis > 37.656341:
                if min_axis <= 39.051248:
                    if rel_mean_depth <= 0.02745:
                        if intensity <= 28.734219:
                            if rel_mean_depth <= 0.02736:
                                return "R20_horizontal"
                            if rel_mean_depth > 0.02736:
                                if intensity <= 18.445988:
                                    if intensity_dif <= 3.214282:
                                        return "V20_horizontal"
                                    if intensity_dif > 3.214282:
                                        return "R20_horizontal"
                                if intensity > 18.445988:
                                    return "R20_horizontal"
                        if intensity > 28.734219:
                            if intensity_dif <= -2.602618:
                                return "R20_horizontal"
                            if intensity_dif > -2.602618:
                                return "V20_horizontal"
                    if rel_mean_depth > 0.02745:
                        if intensity_dif <= -7.994774:
                            if rel_mean_depth <= 0.028205:
                                return "R20_horizontal"
                            if rel_mean_depth > 0.028205:
                                return "V20_horizontal"
                        if intensity_dif > -7.994774:
                            if intensity <= 18.121377:
                                if aspect_ratio <= 1.389137:
                                    return "R20_horizontal"
                                if aspect_ratio > 1.389137:
                                    if intensity <= 10.606241:
                                        return "R20_horizontal"
                                    if intensity > 10.606241:
                                        if rel_mean_depth <= 0.027615:
                                            if min_axis <= 31.256999:
                                                return "R20_horizontal"
                                            if min_axis > 31.256999:
                                                return "V20_horizontal"
                                        if rel_mean_depth > 0.027615:
                                            return "V20_horizontal"
                            if intensity > 18.121377:
                                return "V20_horizontal"
                if min_axis > 39.051248:
                    return "M30_horizontal"
        if max_axis > 66.241981:
            if intensity <= 108.716075:
                if max_axis <= 92.800862:
                    return "F20_20_B_horizontal"
                if max_axis > 92.800862:
                    return "M20_100_horizontal"
            if intensity > 108.716075:
                return "F20_20_G_horizontal"
    if above_laser > 0:
        if area <= 1757:
            if valid_depth_readings <= 408:
                if intensity <= 88.538095:
                    if area <= 1118:
                        if intensity_dif <= -11.417989:
                            if valid_depth_readings <= 143:
                                if rel_max_depth <= 0.68102:
                                    return "R20_vertical"
                                if rel_max_depth > 0.68102:
                                    return "V20_vertical"
                            if valid_depth_readings > 143:
                                if intensity <= 60.007979:
                                    return "R20_vertical"
                                if intensity > 60.007979:
                                    if intensity_dif <= -65.252403:
                                        if area <= 843:
                                            return "R20_vertical"
                                        if area > 843:
                                            return "V20_vertical"
                                    if intensity_dif > -65.252403:
                                        if rel_mean_depth <= 0.014621:
                                            if intensity_dif <= -24.465909:
                                                if intensity <= 68.684145:
                                                    if rel_mean_depth <= 0.000273:
                                                        return "R20_vertical"
                                                    if rel_mean_depth > 0.000273:
                                                        return "V20_vertical"
                                                if intensity > 68.684145:
                                                    return "V20_vertical"
                                            if intensity_dif > -24.465909:
                                                if aspect_ratio <= 1.084886:
                                                    return "R20_vertical"
                                                if aspect_ratio > 1.084886:
                                                    return "V20_vertical"
                                        if rel_mean_depth > 0.014621:
                                            return "R20_vertical"
                        if intensity_dif > -11.417989:
                            if intensity_dif <= 18.873563:
                                if intensity <= 62.164029:
                                    if min_axis <= 22.472205:
                                        return "S40_40_B_horizontal"
                                    if min_axis > 22.472205:
                                        return "M20_100_HD_vertical"
                                if intensity > 62.164029:
                                    if aspect_ratio <= 1.222832:
                                        if intensity <= 80.695:
                                            return "V20_vertical"
                                        if intensity > 80.695:
                                            return "R20_vertical"
                                    if aspect_ratio > 1.222832:
                                        return "F20_20_G_vertical"
                            if intensity_dif > 18.873563:
                                if max_axis <= 35.468296:
                                    if intensity <= 81.862385:
                                        if max_axis <= 34.82815:
                                            if min_axis <= 29.427878:
                                                if intensity_dif <= 35.215184:
                                                    if convex_defects <= 1:
                                                        if aspect_ratio <= 1.24:
                                                            return "V20_vertical"
                                                        if aspect_ratio > 1.24:
                                                            if intensity_dif <= 23.317428:
                                                                return "V20_vertical"
                                                            if intensity_dif > 23.317428:
                                                                return "R20_vertical"
                                                    if convex_defects > 1:
                                                        if intensity <= 41.303922:
                                                            return "V20_vertical"
                                                        if intensity > 41.303922:
                                                            return "M20_100_HD_vertical"
                                                if intensity_dif > 35.215184:
                                                    if min_axis <= 19.235384:
                                                        return "V20_vertical"
                                                    if min_axis > 19.235384:
                                                        if min_axis <= 28.460499:
                                                            if convex_defects <= 1:
                                                                return "R20_vertical"
                                                            if convex_defects > 1:
                                                                if min_axis <= 26.476405:
                                                                    return "V20_vertical"
                                                                if min_axis > 26.476405:
                                                                    return "R20_vertical"
                                                        if min_axis > 28.460499:
                                                            if rel_max_depth <= 0.001003:
                                                                return "R20_vertical"
                                                            if rel_max_depth > 0.001003:
                                                                return "V20_vertical"
                                            if min_axis > 29.427878:
                                                return "V20_vertical"
                                        if max_axis > 34.82815:
                                            if convex_defects <= 0:
                                                return "M20_100_HD_vertical"
                                            if convex_defects > 0:
                                                if intensity <= 47.268868:
                                                    return "V20_vertical"
                                                if intensity > 47.268868:
                                                    if intensity_dif <= 53.092968:
                                                        return "M20_100_HD_vertical"
                                                    if intensity_dif > 53.092968:
                                                        return "V20_vertical"
                                    if intensity > 81.862385:
                                        return "R20_vertical"
                                if max_axis > 35.468296:
                                    if min_axis <= 20.248457:
                                        return "S40_40_B_vertical"
                                    if min_axis > 20.248457:
                                        if intensity <= 74.130653:
                                            return "M20_100_HD_vertical"
                                        if intensity > 74.130653:
                                            return "F20_20_B_vertical"
                    if area > 1118:
                        if valid_depth_readings <= 6:
                            if intensity <= 44.867769:
                                if convex_defects <= 1:
                                    return "M20_100_HU_vertical"
                                if convex_defects > 1:
                                    if area <= 1194.5:
                                        return "M20_100_HD_vertical"
                                    if area > 1194.5:
                                        return "M20_100_HU_vertical"
                            if intensity > 44.867769:
                                return "M20_100_HU_vertical"
                        if valid_depth_readings > 6:
                            if convex_defects <= 1:
                                return "M20_100_HU_vertical"
                            if convex_defects > 1:
                                return "M20_100_HD_vertical"
                if intensity > 88.538095:
                    if valid_depth_readings <= 78:
                        if min_axis <= 20.248457:
                            return "V20_vertical"
                        if min_axis > 20.248457:
                            if intensity <= 113.539033:
                                if intensity_dif <= 57.206929:
                                    if convex_defects <= 1:
                                        if intensity <= 99.142544:
                                            if aspect_ratio <= 1.19544:
                                                return "R20_vertical"
                                            if aspect_ratio > 1.19544:
                                                return "F20_20_B_vertical"
                                        if intensity > 99.142544:
                                            return "F20_20_B_vertical"
                                    if convex_defects > 1:
                                        if max_axis <= 38.288379:
                                            if intensity_dif <= 16.348798:
                                                return "F20_20_B_vertical"
                                            if intensity_dif > 16.348798:
                                                if rel_mean_depth <= 0.67699:
                                                    if aspect_ratio <= 1.236842:
                                                        return "F20_20_G_vertical"
                                                    if aspect_ratio > 1.236842:
                                                        if intensity_dif <= 38.04321:
                                                            return "F20_20_B_vertical"
                                                        if intensity_dif > 38.04321:
                                                            return "F20_20_G_vertical"
                                                if rel_mean_depth > 0.67699:
                                                    return "F20_20_B_vertical"
                                        if max_axis > 38.288379:
                                            return "F20_20_B_vertical"
                                if intensity_dif > 57.206929:
                                    if convex_defects <= 1:
                                        return "R20_vertical"
                                    if convex_defects > 1:
                                        if valid_depth_readings <= 0:
                                            return "F20_20_B_vertical"
                                        if valid_depth_readings > 0:
                                            return "V20_vertical"
                            if intensity > 113.539033:
                                if rel_mean_depth <= 0.676946:
                                    if intensity <= 143.634064:
                                        if area <= 967:
                                            if intensity_dif <= 3.18971:
                                                if intensity <= 134.942492:
                                                    return "F20_20_B_vertical"
                                                if intensity > 134.942492:
                                                    if convex_defects <= 3:
                                                        if valid_depth_readings <= 3:
                                                            return "F20_20_B_vertical"
                                                        if valid_depth_readings > 3:
                                                            return "F20_20_G_vertical"
                                                    if convex_defects > 3:
                                                        return "F20_20_B_vertical"
                                            if intensity_dif > 3.18971:
                                                if convex_defects <= 3:
                                                    if aspect_ratio <= 1.135474:
                                                        if intensity <= 123.160714:
                                                            if aspect_ratio <= 1.085714:
                                                                return "F20_20_G_vertical"
                                                            if aspect_ratio > 1.085714:
                                                                return "F20_20_B_vertical"
                                                        if intensity > 123.160714:
                                                            return "F20_20_B_vertical"
                                                    if aspect_ratio > 1.135474:
                                                        return "F20_20_G_vertical"
                                                if convex_defects > 3:
                                                    if min_axis <= 31.064449:
                                                        if intensity <= 134.393229:
                                                            return "F20_20_G_vertical"
                                                        if intensity > 134.393229:
                                                            return "F20_20_B_vertical"
                                                    if min_axis > 31.064449:
                                                        return "F20_20_B_vertical"
                                        if area > 967:
                                            if max_axis <= 36.400549:
                                                if max_axis <= 36:
                                                    return "F20_20_G_vertical"
                                                if max_axis > 36:
                                                    return "F20_20_B_vertical"
                                            if max_axis > 36.400549:
                                                return "F20_20_G_vertical"
                                    if intensity > 143.634064:
                                        return "F20_20_G_vertical"
                                if rel_mean_depth > 0.676946:
                                    if intensity <= 121.534884:
                                        if min_axis <= 35:
                                            return "F20_20_B_vertical"
                                        if min_axis > 35:
                                            return "F20_20_G_vertical"
                                    if intensity > 121.534884:
                                        return "F20_20_G_vertical"
                    if valid_depth_readings > 78:
                        if aspect_ratio <= 1.030303:
                            return "R20_vertical"
                        if aspect_ratio > 1.030303:
                            if max_axis <= 97.247108:
                                return "V20_vertical"
                            if max_axis > 97.247108:
                                return "F20_20_B_vertical"
            if valid_depth_readings > 408:
                return "M30_vertical"
        if area > 1757:
            if max_axis <= 72.835431:
                if intensity <= 148.502408:
                    if area <= 2150:
                        if intensity <= 112.510903:
                            return "S40_40_B_vertical"
                        if intensity > 112.510903:
                            if valid_depth_readings <= 10:
                                return "S40_40_G_vertical"
                            if valid_depth_readings > 10:
                                if valid_depth_readings <= 13:
                                    return "S40_40_B_vertical"
                                if valid_depth_readings > 13:
                                    return "S40_40_G_vertical"
                    if area > 2150:
                        return "S40_40_B_vertical"
                if intensity > 148.502408:
                    if max_axis <= 56.302753:
                        if area <= 2383:
                            return "S40_40_G_vertical"
                        if area > 2383:
                            if intensity <= 154.691834:
                                return "S40_40_B_vertical"
                            if intensity > 154.691834:
                                return "S40_40_G_vertical"
                    if max_axis > 56.302753:
                        return "S40_40_G_vertical"
            if max_axis > 72.835431:
                if intensity <= 110.662602:
                    return "S40_40_B_horizontal"
                if intensity > 110.662602:
                    return "S40_40_G_horizontal"


def j48_july_rgb(features):
    (area, min_axis, max_axis, aspect_ratio, intensity, intensity_dif, convex_defects, rel_mean_depth, rel_max_depth, valid_depth_readings, above_laser) = features
    if above_laser <= 0:
        if aspect_ratio <= 2.31461:
            if max_axis <= 34.539832:
                if intensity_dif <= -11.314622:
                    return "M20_horizontal"
                if intensity_dif > -11.314622:
                    if max_axis <= 25:
                        return "F20_20_G_horizontal"
                    if max_axis > 25:
                        return "M20_vertical"
            if max_axis > 34.539832:
                if min_axis <= 32.310989:
                    if rel_mean_depth <= 0.027569:
                        if intensity_dif <= 2.314648:
                            if intensity <= 13.262791:
                                return "R20_horizontal"
                            if intensity > 13.262791:
                                if intensity_dif <= -1.805598:
                                    if convex_defects <= 0:
                                        if rel_mean_depth <= 0.027326:
                                            return "R20_horizontal"
                                        if rel_mean_depth > 0.027326:
                                            if rel_mean_depth <= 0.027359:
                                                return "V20_horizontal"
                                            if rel_mean_depth > 0.027359:
                                                if min_axis <= 24.698178:
                                                    return "V20_horizontal"
                                                if min_axis > 24.698178:
                                                    if intensity_dif <= -9.749612:
                                                        return "V20_horizontal"
                                                    if intensity_dif > -9.749612:
                                                        return "R20_horizontal"
                                    if convex_defects > 0:
                                        if max_axis <= 40.261644:
                                            if rel_mean_depth <= 0.026835:
                                                if min_axis <= 25:
                                                    if rel_mean_depth <= 0.026589:
                                                        if max_axis <= 38.639358:
                                                            return "R20_horizontal"
                                                        if max_axis > 38.639358:
                                                            return "V20_horizontal"
                                                    if rel_mean_depth > 0.026589:
                                                        return "R20_horizontal"
                                                if min_axis > 25:
                                                    return "R20_horizontal"
                                            if rel_mean_depth > 0.026835:
                                                if intensity_dif <= -6.91893:
                                                    if valid_depth_readings <= 426:
                                                        return "R20_horizontal"
                                                    if valid_depth_readings > 426:
                                                        return "V20_horizontal"
                                                if intensity_dif > -6.91893:
                                                    return "V20_horizontal"
                                        if max_axis > 40.261644:
                                            return "V20_horizontal"
                                if intensity_dif > -1.805598:
                                    if area <= 1002.5:
                                        return "V20_horizontal"
                                    if area > 1002.5:
                                        return "R20_horizontal"
                        if intensity_dif > 2.314648:
                            if area <= 1009:
                                return "V20_horizontal"
                            if area > 1009:
                                return "R20_horizontal"
                    if rel_mean_depth > 0.027569:
                        return "V20_horizontal"
                if min_axis > 32.310989:
                    if min_axis <= 44.102154:
                        return "M30_horizontal"
                    if min_axis > 44.102154:
                        return "F20_20_G_horizontal"
    if aspect_ratio > 2.31461:
        if convex_defects <= 1:
            if rel_max_depth <= 0.030096:
                if area <= 1205:
                    return "F20_20_G_horizontal"
                if area > 1205:
                    if max_axis <= 91.525953:
                        return "F20_20_B_horizontal"
                    if max_axis > 91.525953:
                        return "M20_100_horizontal"
            if rel_max_depth > 0.030096:
                if rel_max_depth <= 0.082286:
                    return "F20_20_G_horizontal"
                if rel_max_depth > 0.082286:
                    return "M30_horizontal"
        if convex_defects > 1:
            if min_axis <= 22.847319:
                return "F20_20_G_horizontal"
            if min_axis > 22.847319:
                return "M20_100_horizontal"
    if above_laser > 0:
        if valid_depth_readings <= 293:
            if intensity_dif <= -9.638693:
                if convex_defects <= 2:
                    if valid_depth_readings <= 161:
                        if intensity <= 104.776706:
                            if max_axis <= 32.649655:
                                if area <= 649:
                                    return "R20_vertical"
                                if area > 649:
                                    if intensity_dif <= -68.449316:
                                        return "R20_vertical"
                                    if intensity_dif > -68.449316:
                                        return "V20_vertical"
                            if max_axis > 32.649655:
                                if valid_depth_readings <= 102:
                                    if intensity_dif <= -66.952732:
                                        return "R20_vertical"
                                    if intensity_dif > -66.952732:
                                        return "V20_vertical"
                                if valid_depth_readings > 102:
                                    return "V20_vertical"
                        if intensity > 104.776706:
                            return "V20_vertical"
                    if valid_depth_readings > 161:
                        if intensity <= 92.03548:
                            if convex_defects <= 0:
                                if rel_mean_depth <= -0.000433:
                                    return "R20_vertical"
                                if rel_mean_depth > -0.000433:
                                    if rel_mean_depth <= 0.000517:
                                        return "V20_vertical"
                                    if rel_mean_depth > 0.000517:
                                        return "R20_vertical"
                            if convex_defects > 0:
                                return "R20_vertical"
                        if intensity > 92.03548:
                            if rel_max_depth <= 0.007759:
                                return "V20_vertical"
                            if rel_max_depth > 0.007759:
                                if intensity_dif <= -28.121841:
                                    return "R20_vertical"
                                if intensity_dif > -28.121841:
                                    return "V20_vertical"
                if convex_defects > 2:
                    if min_axis <= 36.249138:
                        return "F20_20_G_vertical"
                    if min_axis > 36.249138:
                        return "S40_40_B_vertical"
            if intensity_dif > -9.638693:
                if min_axis <= 26.305893:
                    if intensity <= 33.64532:
                        if convex_defects <= 2:
                            if intensity <= 12.571795:
                                if convex_defects <= 1:
                                    return "V20_vertical"
                                if convex_defects > 1:
                                    return "F20_20_G_vertical"
                            if intensity > 12.571795:
                                if intensity_dif <= 18.35472:
                                    return "M20_100_HD_vertical"
                                if intensity_dif > 18.35472:
                                    if convex_defects <= 1:
                                        return "V20_vertical"
                                    if convex_defects > 1:
                                        return "M20_100_HD_vertical"
                        if convex_defects > 2:
                            return "F20_20_B_vertical"
                    if intensity > 33.64532:
                        if aspect_ratio <= 1.796724:
                            if convex_defects <= 1:
                                if min_axis <= 22.847319:
                                    return "F20_20_B_vertical"
                                if min_axis > 22.847319:
                                    return "V20_vertical"
                            if convex_defects > 1:
                                if intensity <= 87.958333:
                                    return "F20_20_B_vertical"
                                if intensity > 87.958333:
                                    if area <= 496:
                                        return "F20_20_B_vertical"
                                    if area > 496:
                                        if valid_depth_readings <= 62:
                                            return "F20_20_G_vertical"
                                        if valid_depth_readings > 62:
                                            if intensity <= 121.184579:
                                                return "F20_20_B_vertical"
                                            if intensity > 121.184579:
                                                return "F20_20_G_vertical"
                        if aspect_ratio > 1.796724:
                            if max_axis <= 104:
                                if aspect_ratio <= 3.037661:
                                    if convex_defects <= 1:
                                        if area <= 368.5:
                                            if max_axis <= 33.615473:
                                                return "F20_20_G_vertical"
                                            if max_axis > 33.615473:
                                                return "S40_40_B_vertical"
                                        if area > 368.5:
                                            if max_axis <= 46.097722:
                                                return "S40_40_G_vertical"
                                            if max_axis > 46.097722:
                                                return "F20_20_G_vertical"
                                    if convex_defects > 1:
                                        if max_axis <= 44.294469:
                                            if rel_mean_depth <= -0.000343:
                                                return "S40_40_G_horizontal"
                                            if rel_mean_depth > -0.000343:
                                                return "F20_20_B_vertical"
                                        if max_axis > 44.294469:
                                            return "F20_20_G_vertical"
                                if aspect_ratio > 3.037661:
                                    if intensity <= 73.254065:
                                        if valid_depth_readings <= 119:
                                            if intensity_dif <= -3.584351:
                                                if max_axis <= 103:
                                                    return "S40_40_G_vertical"
                                                if max_axis > 103:
                                                    return "F20_20_G_vertical"
                                            if intensity_dif > -3.584351:
                                                return "S40_40_G_vertical"
                                        if valid_depth_readings > 119:
                                            return "S40_40_B_vertical"
                                    if intensity > 73.254065:
                                        return "S40_40_B_vertical"
                            if max_axis > 104:
                                return "F20_20_G_vertical"
                if min_axis > 26.305893:
                    if intensity <= 59.533835:
                        if intensity <= 20.587444:
                            return "M20_100_HD_vertical"
                        if intensity > 20.587444:
                            return "M20_100_HU_vertical"
                    if intensity > 59.533835:
                        if valid_depth_readings <= 50:
                            if max_axis <= 31.827661:
                                return "F20_20_B_vertical"
                            if max_axis > 31.827661:
                                return "S40_40_B_vertical"
                        if valid_depth_readings > 50:
                            if area <= 1178.5:
                                return "F20_20_G_vertical"
                            if area > 1178.5:
                                if convex_defects <= 2:
                                    return "F20_20_G_vertical"
                                if convex_defects > 2:
                                    if max_axis <= 50.219518:
                                        return "S40_40_B_vertical"
                                    if max_axis > 50.219518:
                                        return "S40_40_G_vertical"
        if valid_depth_readings > 293:
            if min_axis <= 29.832868:
                if rel_max_depth <= 0.038227:
                    if max_axis <= 66:
                        return "V20_vertical"
                    if max_axis > 66:
                        return "S40_40_G_horizontal"
                if rel_max_depth > 0.038227:
                    return "M30_vertical"
            if min_axis > 29.832868:
                if intensity <= 81.458333:
                    if min_axis <= 52:
                        return "S40_40_B_horizontal"
                    if min_axis > 52:
                        if min_axis <= 55.00909:
                            return "S40_40_G_horizontal"
                        if min_axis > 55.00909:
                            return "F20_20_G_vertical"
                if intensity > 81.458333:
                    if max_axis <= 36.400549:
                        return "V20_vertical"
                    if max_axis > 36.400549:
                        if rel_max_depth <= 0.005582:
                            if valid_depth_readings <= 605:
                                return "S40_40_B_vertical"
                            if valid_depth_readings > 605:
                                return "S40_40_G_horizontal"
                        if rel_max_depth > 0.005582:
                            if max_axis <= 96.150923:
                                return "S40_40_G_vertical"
                            if max_axis > 96.150923:
                                if intensity <= 104.776706:
                                    return "S40_40_B_vertical"
                                if intensity > 104.776706:
                                    return "S40_40_G_vertical"