# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(slaw_arm_navigation)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    brics_actuator
    control_msgs
    dynamic_reconfigure
    geometry_msgs
    moveit_msgs
    roscpp
    rospy
    sensor_msgs
    trajectory_msgs
    slaw_actions
    slaw_srvs)

generate_dynamic_reconfigure_options(
    cfg/ArmConf.cfg
)


# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
## Generate added messages and services with any dependencies listed here

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS 
    CATKIN_DEPENDS slaw_srvs slaw_actions
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
