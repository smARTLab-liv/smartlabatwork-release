#ifndef EXTRINSIC_CAMERA_CALIBRATION_H
#define EXTRINSIC_CAMERA_CALIBRATION_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <CameraCalibration/camera_calibration_utils.h>

#include <slaw_object_detection/CamCalibrExtrConfig.h>
#include <dynamic_reconfigure/server.h>


using namespace cv;
using namespace std;

namespace slaw_object_detection{



class ExtrinsicCameraCalibration
{
public:
    ExtrinsicCameraCalibration(int boardCols, int boardRows, double fieldSize);

private:

    void processImage(const sensor_msgs::ImageConstPtr &ms);

    void calcChessPosition(vector<Point3d> *boardPoints3D, vector<Point2d> *imagePoints, Mat &rvec, Mat &tvec);

    void drawTransRotText(Mat &mat, Mat &rvec, Mat &tvec);

    void drawAxisOnChessBoard(Mat &img, Mat &rvec, Mat &tvec, double axisLength, double offsetX, double offsetY);

    void initializeCameraParameter();

    void updateCameraParameters();

    void readConfigFile(CamCalibrExtrConfig &config);

    void saveYamlFile();

    void updateConfig(CamCalibrExtrConfig &config, uint32_t level);

    void getBirdEyeImage(Mat &src, Mat &dst, vector<Point3d> &object, vector<Point2d> &image);

    void rotateImage(const Mat &input, Mat &output, double alpha, double beta, double gamma, double dx, double dy, double dz, double f);

    ros::NodeHandle m_nh;
    Size m_boardSize;
    double m_fieldSize;

    Mat m_intrinsics;
    Mat m_distortion;

    image_transport::ImageTransport m_image_transport;
    image_transport::Subscriber m_image_sub;

    CameraCalibrationUtils::IPMInfo m_ipmInfo;
    CameraCalibrationUtils::CameraInfo m_cameraInfo;

    CamCalibrExtrConfig m_dynConfig;
    dynamic_reconfigure::Server<CamCalibrExtrConfig> m_dyn_conf_server;
    dynamic_reconfigure::Server<CamCalibrExtrConfig>::CallbackType m_callback;

    vector<Point3d> m_boardPoints3D;

    vector<int> m_imgIndex;

    string m_yamlFileName;

    bool m_isConfigured;

    double m_alpha, m_beta, m_gamma, m_dx, m_dy, m_dz, m_f;
};
}

#endif // CAMERA_INTRINSIC_CALIBRATION_H
