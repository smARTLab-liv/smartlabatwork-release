# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(slaw_location_service)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin
    REQUIRED COMPONENTS 
    rospy 
    tf
    slaw_msgs
    slaw_srvs
)

include_directories(${Boost_INCLUDE_DIRS})

catkin_package(
    DEPENDS rospy tf
    CATKIN_DEPENDS slaw_msgs slaw_srvs
    INCLUDE_DIRS
    LIBRARIES
)
