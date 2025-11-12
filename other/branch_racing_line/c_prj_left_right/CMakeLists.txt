cmake_minimum_required (VERSION 4.0)
project(example LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED)

add_executable (example DistinguishLeftRight.cpp
        DistinguishLeftRight.h)

target_include_directories(example PRIVATE ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries (example Eigen3::Eigen)
