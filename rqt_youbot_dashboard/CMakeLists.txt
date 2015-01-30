cmake_minimum_required(VERSION 2.8.3)
project(rqt_youbot_dashboard)

find_package(catkin REQUIRED COMPONENTS rospy
                                        rqt_robot_dashboard
)

catkin_python_setup()

catkin_package()

install(PROGRAMS scripts/rqt_youbot_dashboard
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
