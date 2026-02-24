cmake_minimum_required(VERSION 3.10)
project(imu VERSION 0.1.0 LANGUAGES CXX)

# =====================================
# ⚙️ Configuración general
# =====================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# =====================================
# 📂 Rutas de inclusión (headers)
# =====================================
include_directories(
    /usr/include/eigen3
    ${PROJECT_SOURCE_DIR}/include
)

# =====================================
# 📄 Fuente principal del programa
# =====================================
add_executable(imu_reader
    src/main.cpp
)

# =====================================
# 🔗 Enlaces a librerías del sistema
# =====================================
# Estas vienen por defecto en RPi OS
target_link_libraries(imu_reader
    PRIVATE
    pthread   # Hilos (si lo usas más adelante)
    m         # Librería matemática
)

# =====================================
# 🪄 Mensaje informativo de compilación
# =====================================
message(STATUS "Compilando IMU Reader para Raspberry Pi")
