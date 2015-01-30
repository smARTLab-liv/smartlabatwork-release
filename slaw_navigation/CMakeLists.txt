cmake_minimum_required(VERSION 2.8.3)
project(slaw_navigation)

find_package(catkin
    REQUIRED COMPONENTS
    rospy
    dynamic_reconfigure
    tf
    slaw_pid
    slaw_actions
    slaw_msgs
    slaw_srvs
    )


generate_dynamic_reconfigure_options(
  cfg/pid.cfg
)

catkin_package(DEPENDS
    CATKIN_DEPENDS
    slaw_actions
    slaw_srvs
    slaw_msgs)


include_directories(${catkin_INCLUDE_DIRS})

add_executable(lowpass_filter src/lowpass_filter.cpp)
target_link_libraries(lowpass_filter ${catkin_LIBRARIES})
