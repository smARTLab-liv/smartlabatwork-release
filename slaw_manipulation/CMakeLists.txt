# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(slaw_manipulation)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    brics_actuator
    control_msgs
    geometry_msgs
    moveit_msgs
    roscpp
    rospy
    sensor_msgs
    slaw_actions
    slaw_msgs
    slaw_srvs
    tf
    trajectory_msgs)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS slaw_msgs  slaw_srvs slaw_actions
)
