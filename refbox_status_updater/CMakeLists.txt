# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(refbox_status_updater)
find_package(catkin
    REQUIRED COMPONENTS
    rospy
    std_msgs
)

# catkin_python_setup()

catkin_package(CATKIN_DEPENDS
        rospy
        std_msgs
)
