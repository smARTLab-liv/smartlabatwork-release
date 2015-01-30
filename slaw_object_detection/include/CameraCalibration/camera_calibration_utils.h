#ifndef CAMERACALIBRATIONUTILS_H
#define CAMERACALIBRATIONUTILS_H


#define DATA_STR "data"
#define COLS_STR "cols"
#define ROWS_STR "rows"

#include <string>
#include <cv.h>
#include <highgui.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>

#include <exception>

/**
  * Static method which offers distance calculations
  * Chessboard detection and extracts Matrix(Arrays) of parameters
  */
using namespace std;
using namespace cv;

namespace CameraCalibrationUtils
{

//constant definitions
#define FLOAT_MAT_TYPE CV_32FC1
#define FLOAT_MAT_ELEM_TYPE float

#define INT_MAT_TYPE CV_8UC1
#define INT_MAT_ELEM_TYPE unsigned char

#define FLOAT_IMAGE_TYPE IPL_DEPTH_32F
#define FLOAT_IMAGE_ELEM_TYPE float

#define INT_IMAGE_TYPE IPL_DEPTH_8U
#define INT_IMAGE_ELEM_TYPE unsigned char

#define FLOAT_POINT2D CvPoint2D32f
#define FLOAT_POINT2D_F cvPoint2D632f

#define FLOAT float
#define INT int
#define SHORT_INT unsigned char


/**
 * Structure to hold the info about IPM transformation
 */
typedef struct IPMInfo
{
    ///min and max x-value on ground in world coordinates
    double xLimits[2];
    ///min and max y-value on ground in world coordinates
    double yLimits[2];
    ///conversion between mm in world coordinate on the ground
    ///in x-direction and pixel in image
    double xScale;
    ///conversion between mm in world coordinate on the ground
    ///in y-direction and pixel in image
    double yScale;
    ///width
    int width;
    ///height
    int height;
    ///portion of image height to add to y-coordinate of
    ///vanishing point
    double vpPortion;
    ///Left point in original image of region to make IPM for
    double ipmLeft;
    ///Right point in original image of region to make IPM for
    double ipmRight;
    ///Top point in original image of region to make IPM for
    double ipmTop;
    ///Bottom point in original image of region to make IPM for
    double ipmBottom;
    ///interpolation to use for IPM (0: bilinear, 1:nearest neighbor)
    int ipmInterpolation;
}IPMInfo;

///Camera Calibration info
typedef struct CameraInfo
{
    ///focal length in x and y
    Point2d focalLength;
    ///optical center coordinates in image frame (origin is (0,0) at top left)
    Point2d opticalCenter;
    ///height of camera above ground
    double cameraHeight;
    ///pitch angle in radians (+ve downwards)
    double pitch;
    ///yaw angle in radians (+ve clockwise)
    double yaw;
    ///width of images
    double imageWidth;
    ///height of images
    double imageHeight;
}CameraInfo;


static const string DISTORTION_STR ="distortion_coefficients";
static const string CAMERA_MATRIX_STR = "camera_matrix";


Mat getMat64F_FromParam(ros::NodeHandle &nh, string param);

vector<Point3d> generate3DChessboard(int boardWidth, int boardHeight, double fieldSize);

void mcvGetIPM(const CvMat* inImage, CvMat* outImage,
               IPMInfo *ipmInfo, const CameraInfo *cameraInfo,
               list<CvPoint>* outPoints=NULL);

FLOAT_POINT2D mcvGetVanishingPoint(const CameraInfo *cameraInfo);

void mcvTransformImage2Ground(const CvMat *inPoints,
                              CvMat *outPoints, const CameraInfo *cameraInfo);

void mcvTransformGround2Image(const CvMat *inPoints,
                              CvMat *outPoints, const CameraInfo *cameraInfo);

void mcvIplImageToCvMat(IplImage* im, CvMat **clrImage, CvMat** channelImage);

template<typename T>
void interpolation(const CvMat *inImage, CvMat *outImage, CvMat *uvGrid, int outCol, int outRow,const IPMInfo *ipmInfo, list<CvPoint>* outPoints=NULL);


}

#endif // CAMERACALIBRATIONUTILS_H
