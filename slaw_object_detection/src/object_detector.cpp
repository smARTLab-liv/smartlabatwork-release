#include <object_detector.h>

namespace slaw_object_detection
{

static const std::string OPENCV_WINDOW ="image window";

ObjectDetector::ObjectDetector() : img_transport_(nh_)
{



    image_sub_ = img_transport_.subscribe("/camera/rgb/image_rect_color", 1, &ObjectDetector::progressImage, this);

    callback = boost::bind(&ObjectDetector::updateConfig, this,_1,_2);
    dyn_conf_server_.setCallback(callback);

    cv::namedWindow(OPENCV_WINDOW);
}
ObjectDetector::~ObjectDetector()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

void ObjectDetector::progressImage(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
}

void ObjectDetector::updateConfig(DetectorConfig &config, uint32_t level)
{
    config_ = config;
    image_sub_ = img_transport_.subscribe(config_.image_topic, 1, &ObjectDetector::progressImage, this);
}

}

int main(int argc, char** argv){
    ros::init(argc, argv, "ObjectDetector");

    slaw_object_detection::ObjectDetector detector;

    ros::spin();
    return 0;
}
