cmake_minimum_required(VERSION 3.0.2)
project(camera)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_msgs geometry_msgs cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/CameraSubscriber.py
  src/CameraAdjust.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}  
)

