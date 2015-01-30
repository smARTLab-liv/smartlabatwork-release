# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(slaw_registration)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin
    REQUIRED COMPONENTS
    laser_geometry
    pcl_ros
    roscpp
    slaw_actions
    slaw_srvs
    slaw_msgs
    tf
)




find_package(PCL REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})



catkin_package(
    CATKIN_DEPENDS  slaw_srvs slaw_msgs slaw_actions
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)


add_executable(icp src/laser_merger_icp.cpp)
add_executable(forcefield src/forcefield_recovery.cpp)
add_executable(line src/ransac_line.cpp)

add_dependencies(icp slaw_srvs_gencpp slaw_msgs_genccp slaw_actios_gencpp)
add_dependencies(forcefield slaw_srvs_gencpp slaw_msgs_genccp slaw_actios_gencpp)
add_dependencies(line slaw_srvs_gencpp slaw_msgs_genccp slaw_actios_gencpp)


target_link_libraries(icp ${catkin_LIBRARIES})
target_link_libraries(forcefield ${catkin_LIBRARIES})
target_link_libraries(line ${catkin_LIBRARIES})

