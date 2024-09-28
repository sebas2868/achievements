cmake_minimum_required(VERSION 3.0.2)
project(my_robot_controller)


find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  genmsg
  turtlesim
  geometry_msgs
  nav_msgs
)


#
#catkin_package(

include_directories(
  ${catkin_INCLUDE_DIRS}
)

generate_messages(
    DEPENDENCIES
    std_msgs
    nav_msgs
)

add_executable(publisher_node src/simple_publisher.cpp)
add_executable(subscriber_node src/simple_subscriber.cpp)
add_executable(neural_network_node src/neural_network.cpp)

#Nodos Ros1 gripper
add_executable(gripper_suscriber src/gripper_suscriber.cpp)
add_executable(gripper_publisher src/gripper_publisher.cpp)

add_dependencies(publisher_node ${publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(subscriber_node ${subscriber_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(neural_network_node ${subscriber_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#Nodos Ros1 gripper
add_dependencies(gripper_suscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(gripper_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



target_link_libraries(subscriber_node
${catkin_LIBRARIES}
)

target_link_libraries(publisher_node
${catkin_LIBRARIES}
)

target_link_libraries(neural_network_node
${catkin_LIBRARIES}
)

#Nodos Ros1 gripper
target_link_libraries(gripper_suscriber ${catkin_LIBRARIES})
target_link_libraries(gripper_publisher ${catkin_LIBRARIES})


catkin_install_python(PROGRAMS src/read_esp.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
