cmake_minimum_required(VERSION 3.14)
project(KalmanFilterProject LANGUAGES CXX)

# ------------------------------------------------------------------------------
# VCPKG Toolchain (optional)
# ------------------------------------------------------------------------------
if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "Vcpkg toolchain file")
endif()

# ------------------------------------------------------------------------------
# Compiler Settings
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -pedantic)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(BUILD_SHARED_LIBS OFF)

# ------------------------------------------------------------------------------
# Package Dependencies
# ------------------------------------------------------------------------------
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

# ------------------------------------------------------------------------------
# Include Directories
# ------------------------------------------------------------------------------
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ------------------------------------------------------------------------------
# Kalman_RLS Static Library
# ------------------------------------------------------------------------------
add_library(kalman_filter STATIC
    src/kalman_filter.cpp
)

target_include_directories(kalman_filter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(kalman_filter PUBLIC Eigen3::Eigen)

# ------------------------------------------------------------------------------
# Executable: main_sim_RLS
# ------------------------------------------------------------------------------
add_executable(kalman_sim
    kalman_sim.cpp
)

target_link_libraries(kalman_sim
    PRIVATE
    OpenGL::GL
    glfw
    kalman_filter
)

