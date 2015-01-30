cmake_minimum_required(VERSION 2.8.3)
project(slaw_srvs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  slaw_msgs
  message_generation
  std_msgs
  std_srvs
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   #from slaw_location srvs
   BackplateStateStatus.srv
   BackplateStateModify.srv
   CamPose.srv #from cameramount
   CanPlaceObject.srv
   CheckRV20.srv
   DeleteLocation.srv
   GetBestPlaceLocation.srv
   GetLocation.srv
   GetDist.srv
   GetAngle.srv
   GetObjectAtLocation.srv
   RemoveObjectAtLocation.srv
   SimpleIkSolver.srv #from nav
   StoreLocation.srv
   setOffset.srv #from registration
   setRegistrationTar.srv #from registraiont
   SetSide.srv
   SwitchOnForLocation.srv
   switchOff.srv #from mani
)

   
## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )
## DEPENDS: system dependencies of this project that dependent projects also need


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   slaw_msgs
   sensor_msgs
   std_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES slaw_srvs
   CATKIN_DEPENDS geometry_msgs slaw_msgs message_runtime std_msgs std_srvs
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
