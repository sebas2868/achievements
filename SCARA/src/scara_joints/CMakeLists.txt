cmake_minimum_required(VERSION 3.8)
project(scara_joints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python scripts
install(PROGRAMS
  src/joint_publisher.py
  src/interfaz_usuario.py
  src/comunicacion_arduino.py
  src/inversa_prueba.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install necessary files
install(FILES src/wow.gif
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()