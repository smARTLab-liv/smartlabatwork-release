# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(slaw_ppt)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin
    REQUIRED COMPONENTS
    move_base
    rospy
    slaw_bnt
    slaw_manipulation
    slaw_navigation
    smach
    smach_ros
    visualization_msgs
)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#target_link_libraries(${PROJECT_NAME} another_library)
#find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

catkin_package(
    DEPENDS
    rospy
    smach
    smach_ros
    move_base
    visualization_msgs
    slaw_bnt
    slaw_navigation
    slaw_manipulation
)
