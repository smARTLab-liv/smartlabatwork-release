# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(slaw_btt)
find_package(catkin REQUIRED COMPONENTS
    rospy
    smach
    smach_ros
    move_base
    visualization_msgs
    slaw_bnt
    slaw_navigation
    slaw_manipulation)

catkin_package(
    CATKIN_DEPENDS rospy smach smach_ros move_base visualization_msgs slaw_bnt slaw_navigation slaw_manipulation
)
