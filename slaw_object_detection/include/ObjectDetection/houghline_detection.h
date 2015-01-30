#ifndef HOUGHLINE_DETECTION_H
#define HOUGHLINE_DETECTION_H

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

#include <slaw_object_detection/HoughLineConfig.h>
#include <dynamic_reconfigure/server.h>

using namespace cv;
using namespace std;

namespace slaw_object_detection{

class HoughLineDetection
{
public:
    HoughLineDetection();

private:

    void processImage(const sensor_msgs::ImageConstPtr &ms);

    void initConfig();

    void updateParams();
    void updateMask();

    void readConfigFile(HoughLineConfig &config);

    void saveYamlFile();

    void updateConfig(HoughLineConfig &config, uint32_t level);

    ros::NodeHandle m_nh;

    ros::Publisher m_rv20_pub;
    
    image_transport::ImageTransport m_image_transport;
    image_transport::Subscriber m_image_sub;

    HoughLineConfig m_dynConfig;
    dynamic_reconfigure::Server<HoughLineConfig> m_dyn_conf_server;
    dynamic_reconfigure::Server<HoughLineConfig>::CallbackType m_callback;

    string m_yamlFileName;

    bool m_isConfigured;

    boost::mutex m_configure_mutex;

    Mat m_mask;

    int m_mask_x;
    int m_mask_y;
    int m_mask_width;
    int m_mask_height;

    double m_hough_rho;
    double m_hough_theta;
    double m_hough_thresh;
    double m_hough_minLenghth;
    double m_hough_maxLineGap;
    double m_canny_high;
    int m_min_v20;
};
}

#endif // CAMERA_INTRINSIC_CALIBRATION_H
