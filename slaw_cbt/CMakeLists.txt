cmake_minimum_required(VERSION 2.8.3)
project(slaw_cbt)

find_package(catkin 
REQUIRED COMPONENTS
rospy)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_package(
    DEPENDS rospy
)
