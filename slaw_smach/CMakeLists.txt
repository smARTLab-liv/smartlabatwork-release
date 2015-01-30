# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(slaw_smach)
find_package(catkin
    REQUIRED COMPONENTS
    move_base
    rospy
    slaw_arm_navigation
    slaw_location_service
    slaw_manipulation
    slaw_registration
    smach
    smach_ros
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
        rospy
        smach
        smach_ros
        move_base
        slaw_arm_navigation
        slaw_location_service
        slaw_registration
        slaw_manipulation
)
