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
# IMGUI + IMPLOT Integration
# ------------------------------------------------------------------------------
file(GLOB IMGUI_SOURCES
    external/imgui/*.cpp
    external/imgui/backends/imgui_impl_glfw.cpp
    external/imgui/backends/imgui_impl_opengl3.cpp
)

file(GLOB IMPLOT_SOURCES
    external/implot/*.cpp
)

# ------------------------------------------------------------------------------
# Include Directories
# ------------------------------------------------------------------------------
include_directories(
    external/imgui
    external/imgui/backends
    external/implot
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ------------------------------------------------------------------------------
# Kalman_RLS Static Library
# ------------------------------------------------------------------------------
add_library(kalman_RLS STATIC
    src/Kalman_RLS.cpp
)

target_include_directories(kalman_RLS PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(kalman_RLS PUBLIC Eigen3::Eigen)

# ------------------------------------------------------------------------------
# Executable: main_sim_RLS
# ------------------------------------------------------------------------------
add_executable(main_sim_RLS 
    main_sim_RLS.cpp
    ${IMGUI_SOURCES}
    ${IMPLOT_SOURCES}
)

target_link_libraries(main_sim_RLS
    PRIVATE
    OpenGL::GL
    glfw
    kalman_RLS
)

