# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(slaw_joystick)
find_package(catkin
    REQUIRED COMPONENTS
    brics_actuator
    geometry_msgs
    roscpp
    sensor_msgs
    tf)


catkin_package(
    DEPENDS
    CATKIN_DEPENDS geometry_msgs sensor_msgs roscpp brics_actuator tf
    INCLUDE_DIRS
    LIBRARIES
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(stick src/youbot_joystick.cpp)
target_link_libraries(stick ${catkin_LIBRARIES})
