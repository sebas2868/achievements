cmake_minimum_required(VERSION 3.10)
project(cinematica_inversa VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ================================
# 🔎 ENCONTRAR LIBRERÍAS EXTERNAS
# ================================
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUV_PKGCONFIG REQUIRED libuv)

# Buscar WiringPi en el sistema
find_library(WIRINGPI_LIB NAMES wiringPi REQUIRED)

# ================================
# 📦 Librería cinemática (modular)
# ================================
add_library(kinematics_lib
    src/kinematics.cpp
    src/BusServoControl.cpp
)

target_include_directories(kinematics_lib PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

# Vinculamos WiringPi a la librería que maneja los servos
target_link_libraries(kinematics_lib PUBLIC ${WIRINGPI_LIB})

# ================================
# 🧩 Ejecutable principal
# ================================
add_executable(inverse_kinematics
    src/main.cpp
)

# Directorios de inclusión de libuv
target_include_directories(inverse_kinematics PRIVATE
    ${LIBUV_PKGCONFIG_INCLUDE_DIRS}
)

# Vincular dependencias al ejecutable
# Al vincular kinematics_lib, ya arrastra WiringPi porque la pusimos como PUBLIC
target_link_libraries(inverse_kinematics PRIVATE 
    kinematics_lib
    ${LIBUV_PKGCONFIG_LIBRARIES}
    pthread  # WiringPi a veces requiere pthread explícitamente
)