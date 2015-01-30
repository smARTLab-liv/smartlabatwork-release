cmake_minimum_required(VERSION 2.8.3)
project(slaw_object_detection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")


find_package(catkin REQUIRED COMPONENTS
  brics_actuator
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  rospy
  slaw_actions
  slaw_msgs
  slaw_srvs
  std_msgs
  tf
)

find_package(OpenCV REQUIRED)


include_directories(
    include
    include/ObjectDetection
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIR}
)

generate_dynamic_reconfigure_options(
    cfg/slaw_camera_calibration.cfg
    cfg/cam_calbr_extrinsic.cfg 
    cfg/slaw_object_detection.cfg
    cfg/hough_circles.cfg
    cfg/hough_line.cfg
)


catkin_package(
  INCLUDE_DIRS include include/ObjectDetection
#  LIBRARIES slaw_object_detection
  CATKIN_DEPENDS
  brics_actuator
  dynamic_reconfigure
  roscpp
  rospy
  slaw_actions
  slaw_msgs
  slaw_srvs
  std_msgs
  tf
  DEPENDS OpenCV
)

add_executable(ObjectDetector src/object_detector.cpp)
add_executable(HoughCircles src/ObjectDetection/hough_circles_detection.cpp)
add_executable(CameraCalibration src/camera_calibration.cpp)
add_executable(CameraCalibration2 src/CameraCalibration/camera_calibration.cpp src/CameraCalibration/settings.cpp)
add_executable(ExtrinsicParameter ExtrinsicCameraParams.cpp)
add_executable(ExtrinsicCalibr src/CameraCalibration/extrinsic_camera_calibration.cpp src/CameraCalibration/camera_calibration_utils.cpp)
add_executable(HoughLine src/ObjectDetection/houghline_detection.cpp)

add_dependencies(HoughCircles ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(ObjectDetector ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(CameraCalibration2 ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(CameraCalibration ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(ExtrinsicParameter ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(ExtrinsicCalibr ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(HoughLine ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(HoughCircles ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
target_link_libraries(ObjectDetector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(CameraCalibration2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(CameraCalibration ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(ExtrinsicParameter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(ExtrinsicCalibr ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(HoughLine ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_dependencies(ObjectDetector slaw_object_detection_gencfg)
