//#include "/home/ramsi/src/ros/src/smartlabatwork/slaw_object_detection/include/ObjectDetection/circle_hough_detection.h"
#include "ObjectDetection/houghline_detection.h"
#include "std_msgs/String.h"

namespace slaw_object_detection{

HoughLineDetection::HoughLineDetection()
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

    m_callback = boost::bind(&HoughLineDetection::updateConfig, this,_1,_2);
    m_dyn_conf_server.setCallback(m_callback);

    initConfig();

    m_image_sub = m_image_transport.subscribe(img_topic, 1, &HoughLineDetection::processImage, this);

    m_rv20_pub = m_nh.advertise<std_msgs::String>("/slaw_object_detection/rv20_lines", 1);

    ros::spin();
}

void HoughLineDetection::updateConfig(HoughLineConfig &config, uint32_t level)
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

void HoughLineDetection::initConfig()
{
    //initialize dynamic config
    cout << "initConfig!" <<endl;
    //FileStorage fs(m_yamlFileName, FileStorage::READ);

    readConfigFile(m_dynConfig);

    //initConfig
    updateParams();
}

void HoughLineDetection::readConfigFile(HoughLineConfig &config)
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
    fs ["hough_rho"]   >>m_dynConfig.hough_rho;
    fs ["hough_theta"] >> m_dynConfig.hough_theta;
    fs ["hough_thresh"]  >> m_dynConfig.hough_thresh;
    fs [ "hough_minLenghth"] >>m_dynConfig.hough_minLenghth;
    fs ["hough_maxLineGap"] >> m_dynConfig.hough_maxLineGap;
    fs["canny_high"]   >> config.canny_high;
    fs["min_v20"]   >> config.min_v20;
}

void HoughLineDetection::saveYamlFile()
{
    FileStorage fs(m_yamlFileName, FileStorage::WRITE);

    fs << "mask_x" << m_dynConfig.mask_x;
    fs << "mask_y" << m_dynConfig.mask_y;
    fs << "mask_width" << m_dynConfig.mask_width;
    fs << "mask_height" << m_dynConfig.mask_height;
    fs << "hough_rho" << m_dynConfig.hough_rho;
    fs << "hough_theta" << m_dynConfig.hough_theta;
    fs << "hough_thresh" << m_dynConfig.hough_thresh;
    fs << "hough_minLenghth" << m_dynConfig.hough_minLenghth;
    fs << "hough_maxLineGap" << m_dynConfig.hough_maxLineGap;
    fs << "canny_high" << m_dynConfig.canny_high;
    fs << "min_v20" << m_dynConfig.min_v20;
}

void HoughLineDetection::updateParams()
{
    boost::mutex::scoped_lock l(m_configure_mutex);

    m_mask_x = m_dynConfig.mask_x;
    m_mask_y = m_dynConfig.mask_y;
    m_mask_width = m_mask_x + m_dynConfig.mask_width > 640 ? 640 - m_mask_x : m_dynConfig.mask_width;
    m_mask_height = m_mask_y + m_dynConfig.mask_height > 480 ? 480 - m_mask_y : m_dynConfig.mask_height;
    cout << m_mask_x << "," << m_mask_y << "+" << m_mask_width << "," << m_mask_height << endl;

    m_hough_theta = m_dynConfig.hough_theta;
    m_hough_thresh = m_dynConfig.hough_thresh;
    m_hough_minLenghth = m_dynConfig.hough_minLenghth;
    m_hough_maxLineGap = m_dynConfig.hough_maxLineGap;
    m_hough_rho = m_dynConfig.hough_rho;
    m_canny_high = m_dynConfig.canny_high;
    m_min_v20 = m_dynConfig.min_v20;
}

void HoughLineDetection::updateMask()
{
}

void HoughLineDetection::processImage(const sensor_msgs::ImageConstPtr &ms)
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

    GaussianBlur( zero, zero, Size(9, 9), 2, 2 );
   // vector<Vec3f> circles;
    Mat canny;
    Canny(zero, canny, m_canny_high / 2, m_canny_high);
    //HoughCircles(zero, circles, CV_HOUGH_GRADIENT, m_hough_dp, m_hough_dmin,
      //           m_canny_high, m_hough_thresh, m_hough_rmin, m_hough_rmax);


   /*vector<Vec2f> lines;
    HoughLines(canny, lines,m_hough_rho , m_hough_theta,m_hough_thresh, m_hough_minLenghth, m_hough_maxLineGap );


    for( size_t i = 0; i < lines.size(); i++ )
    {
      float rho = lines[i][0], theta = lines[i][1];
      Point pt1, pt2;
      double a = cos(theta), b = sin(theta);
      double x0 = a*rho, y0 = b*rho;
      pt1.x = cvRound(x0 + 1000*(-b));
      pt1.y = cvRound(y0 + 1000*(a));
      pt2.x = cvRound(x0 - 1000*(-b));
      pt2.y = cvRound(y0 - 1000*(a));
      line( zero, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
    }*/

    vector<Vec4i> lines;
    HoughLinesP(canny, lines, m_hough_rho , m_hough_theta,m_hough_thresh, m_hough_minLenghth, m_hough_maxLineGap );

    for( size_t i = 0; i < lines.size(); i++ )
    {
      Vec4i l = lines[i];
      line( zero, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
    }

    //cout << "number of lines :" << lines.size() << endl;

    std_msgs::String msg;

    if (lines.size() > m_min_v20) {
        msg.data = "V20";
    } else {
        msg.data = "R20";
    }
    m_rv20_pub.publish(msg);

    namedWindow( "circles", 1 );
    imshow( "circles", zero );

    namedWindow( "canny", 1 );
    imshow( "canny", canny );

    waitKey(10);
}
}// end namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Intrinsic Cameracalibration");
    ros::NodeHandle nh("~");

    slaw_object_detection::HoughLineDetection();
}

