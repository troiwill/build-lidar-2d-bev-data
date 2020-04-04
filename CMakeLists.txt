cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(build_lidar_2d_bev_data)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Bring the headers into the project.
include_directories(include)

# Add the 'src' subdirectory into the CMake building process.
add_subdirectory(src)
