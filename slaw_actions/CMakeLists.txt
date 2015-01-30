cmake_minimum_required(VERSION 2.8.3)
project(slaw_actions)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  slaw_msgs
)

add_action_files(
  FILES
  AlignLineDistance.action
  FineAdjust.action
  GripOrPlace.action
  OdomFineAdjust.action
  Offset.action
  RegistrationFineAdjust.action
  TuckArm.action
  VisualServoing.action
 ) 


generate_messages(
    DEPENDENCIES 
    actionlib_msgs
    control_msgs
    geometry_msgs
    sensor_msgs
    trajectory_msgs
    )
  
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES slaw_actions
   CATKIN_DEPENDS actionlib actionlib_msgs  message_runtime slaw_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

