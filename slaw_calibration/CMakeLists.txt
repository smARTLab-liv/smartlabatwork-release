cmake_minimum_required(VERSION 2.8.3)
project(slaw_calibration)
find_package(catkin REQUIRED COMPONENTS rospy tf roscpp sensor_msgs)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS
    rospy
    roscpp
    tf
    sensor_msgs
)
add_executable(ar_camera_calibrator src/ARCameraCalibrator.cpp)
target_link_libraries(ar_camera_calibrator ${catkin_LIBRARIES})
