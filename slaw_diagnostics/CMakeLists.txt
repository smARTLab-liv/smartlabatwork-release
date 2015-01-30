cmake_minimum_required(VERSION 2.8.3)
project(slaw_diagnostics)
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    slaw_msgs
    rospy)
find_package(PCL REQUIRED)


catkin_package(
    CATKIN_DEPENDS rospy roscpp slaw_msgs 
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(slaw_battery_monitor src/slaw_battery_monitor.cpp nodes/slaw_battery_monitor_node.cpp )
target_link_libraries(slaw_battery_monitor ${catkin_LIBRARIES} stdc++)

add_dependencies(slaw_battery_monitor slaw_msgs_gencpp)

