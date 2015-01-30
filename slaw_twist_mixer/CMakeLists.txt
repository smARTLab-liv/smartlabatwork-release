# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(slaw_twist_mixer)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)

catkin_package(
    DEPENDS roscpp geometry_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

include_directories(${Boost_INCLUDE_DIRS})

add_executable(twist_mixer src/twist_mixer.cpp)
target_link_libraries(twist_mixer ${Boost_LIBRARIES} ${catkin_LIBRARIES})

