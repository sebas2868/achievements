cmake_minimum_required(VERSION 3.10)

# Nombre del Proyecto
project(ControlPS4 VERSION 1.0)

# 1. Configuración del Estándar C++
# Usamos C++17 para características modernas como std::clamp
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 2. Opciones de Compilación
# -O3: Optimización máxima
# -pthread: Necesario para usar hilos (std::thread) en Linux
add_compile_options(-O3 -Wall -pthread)

# 3. Paquetes Requeridos
find_package(Threads REQUIRED)

# 4. Definir el Ejecutable
# Apuntamos a la carpeta 'src' donde está tu código
add_executable(ps4_control src/main.cpp)

# 5. Directorios de Cabecera (Include Directories)
# Si tienes archivos .h o .hpp en 'src', esto permite encontrarlos fácilmente
target_include_directories(ps4_control PRIVATE src)

# 6. Vincular Librerías
# Vinculamos la librería de hilos al ejecutable
target_link_libraries(ps4_control PRIVATE Threads::Threads)

message(STATUS "Configuración lista. Ejecutable será: ps4_control")