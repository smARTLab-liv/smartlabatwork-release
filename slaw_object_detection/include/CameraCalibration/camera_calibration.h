#ifndef CAMERA_CALIBRATION_H
#define CAMERA_CALIBRATION_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <slaw_object_detection/CameraCalibrationConfig.h>
#include <dynamic_reconfigure/server.h>
#include <CameraCalibration/settings.h>



#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

namespace slaw_object_detection
{

class CameraCalibration
{
public:

    const Scalar RED;
    const Scalar GREEN;
    const char ESC_KEY;

public:
    enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

    CameraCalibration();


    static void help()
    {
        cout <<  "This is a camera calibration sample." << endl
              <<  "Usage: calibration configurationFile"  << endl
               <<  "Near the sample file you'll find the configuration file, which has detailed help of "
                   "how to edit it.  It may be any OpenCV supported file format XML/YAML." << endl;
    }

    static void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
    {
        if(node.empty())
            x = default_value;
        else
            x.read(node);
    }


    bool runCalibrationAndSave(Settings& settings_, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
                               vector<vector<Point2f> > imagePoints );


    double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                         const vector<vector<Point2f> >& imagePoints,
                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                         const Mat& cameraMatrix , const Mat& distCoeffs,
                                         vector<float>& perViewErrors);

    void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/);

    bool runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                                vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                                vector<float>& reprojErrs,  double& totalAvgErr);

    void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                              double totalAvgErr );




    private:

    void setup3DChessboardPoints();


    void extrinsicCalculation(vector<Point2f> img_points, vector<Point3f> object_points, Mat *img = NULL);

    void updateConfig(CameraCalibrationConfig &config, uint32_t level);

    void processKeyInput();

    void processImage(const sensor_msgs::ImageConstPtr &ms);

    Settings settings_;

    vector<vector<Point2f> > imagePoints;

    Mat cameraMatrix, distCoeffs;

    Size imageSize;

    int mode;

    clock_t prevTimestamp;

    std::vector<Point3f> chessboard3DCoor_;

    ros::NodeHandle nh_;
    image_transport::ImageTransport img_transport_;
    image_transport::Subscriber image_sub_;
    CameraCalibrationConfig config_;
    dynamic_reconfigure::Server<CameraCalibrationConfig> dyn_conf_server_;
    dynamic_reconfigure::Server<CameraCalibrationConfig>::CallbackType callback;

};
} //end namespace
#endif // CAMERA_CALIBRATION_H
