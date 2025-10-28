cmake_minimum_required(VERSION 4.0)
project(PERCEPTION)

set(CMAKE_CXX_STANDARD 20)

add_executable(perception main.cpp
        pcd_write.cpp)
