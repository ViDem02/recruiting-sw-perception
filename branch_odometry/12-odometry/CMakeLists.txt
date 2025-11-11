cmake_minimum_required(VERSION 4.0)
project(second_prj_setup)

set(CMAKE_CXX_STANDARD 20)

find_package(PCL 1.3 REQUIRED)
# add_executable(sample_consensus main.cpp)
# target_link_libraries(sample_consensus ${PCL_LIBRARIES})

# PCD-based ICP demo using first.pcd (target) and second.pcd (source)
add_executable(icp_pcd original_code.cpp)
target_link_libraries(icp_pcd ${PCL_LIBRARIES})
