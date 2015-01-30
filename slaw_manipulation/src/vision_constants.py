__author__ = 'broecker'

CHECK_LASER = ['M20_h', 'M20_100_HD_v', 'M20_100_HU_v', 'RV20_v']

PIXEL_TO_CM = 8.0

MAX_AREA = 5000

MAX_AREA_MERGE = 1000

MIN_AREA = 100
MAX_LEN = 150
MIN_LEN = 10

# #Filter depth values openni2
MIN = 0.6
MAX = 0.72

# use legacy openni1
OPENNI_LEGACY = False

# CROSS_OFF_X = 0
# CROSS_OFF_Y = 0
CROSS_OFF_X = 10
# CROSS_OFF_Y = -54

CROSS_OFF_Y = -22


FULL_VISUALIZE = False

DIST_BETWEEN_CONTOURS = 0.03  # in m


MIN_POINT_LASER_DIS = 13
MIN_OBJECT_LASER_COUNT = 3


# #crop filter
crop_y = [150, 400]
crop_x = [120, 520]

# #For placeing the windows
size_x = crop_x[1] - crop_x[0] + 50
size_y = crop_y[1] - crop_y[0] + 50
