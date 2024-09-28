cmake_minimum_required(VERSION 3.8)
project(cpp_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# for publisher 
add_executable(talker src/publisher_member_function.cpp)

ament_target_dependencies(talker rclcpp std_msgs nav_msgs)

add_executable(publisher_camera src/camera_subscriber.cpp)

ament_target_dependencies(publisher_camera rclcpp std_msgs nav_msgs geometry_msgs)

add_executable(neural_network src/neural_network.cpp)

ament_target_dependencies(neural_network rclcpp std_msgs nav_msgs geometry_msgs)

install(TARGETS
 talker   # for publisher
 publisher_camera
 neural_network
 DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
