cmake_minimum_required(VERSION 2.8.3)
project(slaw_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  control_msgs
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BackplateObject.msg
  BatteryStatus.msg
  Location.msg
  JointDualState.msg
  NetworkStatus.msg
  Object.msg
  ObjectArray.msg
  PoseStampedLabeled.msg
  PoseStampedLabeledArray.msg
  SysInfo.msg
  SystemStatus.msg
  Target.msg
  TargetArray.msg
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   sensor_msgs
   std_msgs
   diagnostic_msgs
 )

catkin_package(
   CATKIN_DEPENDS
   control_msgs
   geometry_msgs 
   message_runtime 
   sensor_msgs 
   std_msgs
   diagnostic_msgs
)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

