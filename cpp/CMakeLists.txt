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
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    external/imgui
    external/imgui/backends
    external/implot
)

# ------------------------------------------------------------------------------
# Kalman Filter Static Library
# ------------------------------------------------------------------------------
add_library(kalman_filter STATIC
    src/kalman_filter.cpp
)

target_include_directories(kalman_filter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(kalman_filter PUBLIC Eigen3::Eigen)

# ------------------------------------------------------------------------------
# Executable: kalman_sim
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

# ------------------------------------------------------------------------------
# Executable: kalman_sim_gui (with ImGui/ImPlot visualization)
# ------------------------------------------------------------------------------
add_executable(kalman_sim_gui
    kalman_sim_gui.cpp
    ${IMGUI_SOURCES}
    ${IMPLOT_SOURCES}
)

target_link_libraries(kalman_sim_gui
    PRIVATE
    OpenGL::GL
    glfw
    kalman_filter
)

