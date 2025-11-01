cmake_minimum_required(VERSION 4.0)
project(PERCEPTION)

set(CMAKE_CXX_STANDARD 20)

find_package(PCL 1.3 REQUIRED)
add_executable(pcd_write_test pcd_write.cpp)
target_link_libraries(pcd_write_test ${PCL_LIBRARIES})

add_executable(perception main.cpp)


# find_package(PCL 1.3 REQUIRED COMPONENTS io) for specific components