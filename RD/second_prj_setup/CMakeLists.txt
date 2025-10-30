cmake_minimum_required(VERSION 4.0)
project(second_prj_setup)

set(MY_FILE cones.pcd)
set(CMAKE_CXX_STANDARD 20)

find_package(PCL 1.3 REQUIRED)
add_executable(pcd_write_test matrix_transform.cpp)
target_link_libraries(pcd_write_test ${PCL_LIBRARIES})

# Make cube.ply available next to the built executable
set(CUBE_PLY ${CMAKE_CURRENT_SOURCE_DIR}/${MY_FILE})
add_custom_target(copy_my_file ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CUBE_PLY} $<TARGET_FILE_DIR:pcd_write_test>/${MY_FILE}
    DEPENDS ${CUBE_PLY}
)
add_dependencies(pcd_write_test copy_my_file)

# add_executable(second_prj_setup matrix_transform.cpp)
