#include "ObjectDetection/hough_circles_detection.h"
#include "std_msgs/String.h"


namespace slaw_object_detection{

HoughCirclesDetection::HoughCirclesDetection()
    : m_nh("~")
    , m_image_transport(m_nh)
{
    string img_topic;
    if(!m_nh.getParam("image_topic", img_topic))
    {
         ROS_ERROR("PARAMETER : image_topic is not specified");
         exit(1);
    }

    if(!m_nh.getParam("config_file", m_yamlFileName))
    {
         ROS_ERROR("PARAMETER : config_file is not specified");
         exit(1);
    }

    m_callback = boost::bind(&HoughCirclesDetection::updateConfig, this,_1,_2);
    m_dyn_conf_server.setCallback(m_callback);

    initConfig();

    m_image_sub = m_image_transport.subscribe(img_topic, 1, &HoughCirclesDetection::processImage, this);
    m_rv20_pub = m_nh.advertise<std_msgs::String>("/slaw_object_detection/rv20_circles", 1);

    ros::spin();
}

void HoughCirclesDetection::updateConfig(HoughCirclesConfig &config, uint32_t level)
{
    if(!m_isConfigured){
        readConfigFile(config);
        m_isConfigured = true;
    }
    m_dynConfig = config;
    updateParams();
    if(config.save){
        saveYamlFile();
    }
    config.save = false;
    m_dynConfig.save = false;
}

void HoughCirclesDetection::initConfig()
{
    //initialize dynamic config
    cout << "initConfig!" <<endl;
    //FileStorage fs(m_yamlFileName, FileStorage::READ);

    readConfigFile(m_dynConfig);

    //initConfig
    updateParams();
}

void HoughCirclesDetection::readConfigFile(HoughCirclesConfig &config)
{
    //check for file
    ifstream check(m_yamlFileName.c_str());
    if(check.good())check.close();
    else return;

    FileStorage fs(m_yamlFileName, FileStorage::READ);

    fs["mask_x"]       >> config.mask_x;
    fs["mask_y"]       >> config.mask_y;
    fs["mask_width"]   >> config.mask_width;
    fs["mask_height"]  >> config.mask_height;
    fs["hough_dp"]     >> config.hough_dp;
    fs["hough_rmax"]   >> config.hough_rmax;
    fs["hough_rmin"]   >> config.hough_rmin;
    fs["hough_thresh"] >> config.hough_thresh;
    fs["hough_dmin"]   >> config.hough_dmin;
    fs["canny_high"]   >> config.canny_high;
}

void HoughCirclesDetection::saveYamlFile()
{
    FileStorage fs(m_yamlFileName, FileStorage::WRITE);

    fs << "mask_x" << m_dynConfig.mask_x;
    fs << "mask_y" << m_dynConfig.mask_y;
    fs << "mask_width" << m_dynConfig.mask_width;
    fs << "mask_height" << m_dynConfig.mask_height;
    fs << "hough_dp" << m_dynConfig.hough_dp;
    fs << "hough_rmax" << m_dynConfig.hough_rmax;
    fs << "hough_rmin" << m_dynConfig.hough_rmin;
    fs << "hough_thresh" << m_dynConfig.hough_thresh;
    fs << "hough_dmin" << m_dynConfig.hough_dmin;
    fs << "canny_high" << m_dynConfig.canny_high;
}

void HoughCirclesDetection::updateParams()
{
    boost::mutex::scoped_lock l(m_configure_mutex);

    m_mask_x = m_dynConfig.mask_x;
    m_mask_y = m_dynConfig.mask_y;
    m_mask_width = m_mask_x + m_dynConfig.mask_width > 640 ? 640 - m_mask_x : m_dynConfig.mask_width;
    m_mask_height = m_mask_y + m_dynConfig.mask_height > 480 ? 480 - m_mask_y : m_dynConfig.mask_height;
    cout << m_mask_x << "," << m_mask_y << "+" << m_mask_width << "," << m_mask_height << endl;
    m_hough_dp = m_dynConfig.hough_dp;
    m_hough_rmax = m_dynConfig.hough_rmax;
    m_hough_rmin = m_dynConfig.hough_rmin;
    m_hough_thresh = m_dynConfig.hough_thresh;
    m_hough_dmin = m_dynConfig.hough_dmin;
    m_canny_high = m_dynConfig.canny_high;
}

void HoughCirclesDetection::processImage(const sensor_msgs::ImageConstPtr &ms)
{
    if(!m_isConfigured) return;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(ms, ms->encoding);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat gray;
    cvtColor(cv_ptr->image, gray, COLOR_RGB2GRAY);

    m_configure_mutex.lock();
    //  m_mask_x = 150
    //  m_mask_y = 150
    //  m_mask_width = 250
    //  m_mask_height = 250
    Mat zero = gray(Rect(m_mask_x, m_mask_y, m_mask_width, m_mask_height));
    m_configure_mutex.unlock();
    //gray = gray & m_mask;

    GaussianBlur(gray, gray, Size(9, 9), 2, 2 );
    vector<Vec3f> circles;
    Mat canny;
    Canny(zero, canny, m_canny_high / 2, m_canny_high);
    HoughCircles(zero, circles, CV_HOUGH_GRADIENT, m_hough_dp, m_hough_dmin,
                 m_canny_high, m_hough_thresh, m_hough_rmin, m_hough_rmax);

    for( size_t i = 0; i < circles.size(); i++ )
    {
         Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
         int radius = cvRound(circles[i][2]);
         // draw the circle center
         circle( zero, center, 3, Scalar(0,255,0), -1, 8, 0 );
         // draw the circle outline
         circle( zero, center, radius, Scalar(0,0,255), 3, 8, 0 );
    }

    std_msgs::String msg;

 
    if (circles.size() > 0) {
        msg.data = "R20";
    } else {
        msg.data = "V20";
    }
    m_rv20_pub.publish(msg);

    namedWindow("circles", 1);
    imshow("circles", zero );

    namedWindow("canny", 1);
    imshow("canny", canny);

    waitKey(10);
}
}// end namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Intrinsic Cameracalibration");
    ros::NodeHandle nh("~");

    slaw_object_detection::HoughCirclesDetection();
}
