#ifndef HOUGH_H
#define HOUGH_H

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

#include <slaw_object_detection/HoughCirclesConfig.h>
#include <dynamic_reconfigure/server.h>

using namespace cv;
using namespace std;

namespace slaw_object_detection{

class HoughCirclesDetection
{
public:
    HoughCirclesDetection();

private:

    void processImage(const sensor_msgs::ImageConstPtr &ms);

    void initConfig();

    void updateParams();
    void updateMask();

    void readConfigFile(HoughCirclesConfig &config);

    void saveYamlFile();

    void updateConfig(HoughCirclesConfig &config, uint32_t level);

    ros::NodeHandle m_nh;
    ros::Publisher m_rv20_pub;

    image_transport::ImageTransport m_image_transport;
    image_transport::Subscriber m_image_sub;

    HoughCirclesConfig m_dynConfig;
    dynamic_reconfigure::Server<HoughCirclesConfig> m_dyn_conf_server;
    dynamic_reconfigure::Server<HoughCirclesConfig>::CallbackType m_callback;

    string m_yamlFileName;

    bool m_isConfigured;

    boost::mutex m_configure_mutex;

    Mat m_mask;

    int m_mask_x;
    int m_mask_y;
    int m_mask_width;
    int m_mask_height;

    double m_hough_dp;
    int    m_hough_rmax;
    int    m_hough_rmin;
    double m_hough_thresh;
    double m_hough_dmin;

    double m_canny_high;
};
}

#endif // CAMERA_INTRINSIC_CALIBRATION_H
