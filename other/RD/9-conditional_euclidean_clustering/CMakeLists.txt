cmake_minimum_required(VERSION 4.0)
project(second_prj_setup)

set(CMAKE_CXX_STANDARD 20)

find_package(PCL 1.3 REQUIRED)
add_executable(pcd_write_test main.cpp
        ColorUtilities.h)
target_link_libraries(pcd_write_test ${PCL_LIBRARIES})

# add_executable(second_prj_setup matrix_transform.cpp)
