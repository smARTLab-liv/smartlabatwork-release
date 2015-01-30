# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(slaw_bnt)
# Load catkin and all dependencies required for this package
find_package(catkin
    REQUIRED COMPONENTS
    move_base
    rospy
    smach
    smach_ros
    visualization_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    CATKIN_DEPENDS rospy smach smach_ros move_base visualization_msgs
)
