cmake_minimum_required(VERSION 4.0)
project(second_prj_setup)

# set(MY_FILE cones.pcd) deprecated because in the command
# it's still necessary to mention file name.

set(CMAKE_CXX_STANDARD 20)

find_package(PCL 1.3 REQUIRED)
add_executable(pcd_write_test matrix_transform.cpp)
target_link_libraries(pcd_write_test ${PCL_LIBRARIES})

# add_executable(second_prj_setup matrix_transform.cpp)
