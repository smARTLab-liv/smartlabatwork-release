#ifndef OBJECTDETECTOR_H
#define OBJECTDETECTOR_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <slaw_object_detection/DetectorConfig.h>
#include <dynamic_reconfigure/server.h>

namespace slaw_object_detection
{

class ObjectDetector
{
public:
    ObjectDetector();
    ~ObjectDetector();

private:
    /**
      Processes the raw image
    */
    void progressImage(const sensor_msgs::ImageConstPtr& msg);

    void updateConfig(DetectorConfig &config, uint32_t level);

    ros::NodeHandle nh_;
    image_transport::ImageTransport img_transport_;
    image_transport::Subscriber image_sub_;
    DetectorConfig config_;
    dynamic_reconfigure::Server<DetectorConfig> dyn_conf_server_;
    dynamic_reconfigure::Server<DetectorConfig>::CallbackType callback;
};

}
#endif // OBJECTDETECTOR_H
