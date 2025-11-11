cmake_minimum_required(VERSION 4.0)
project(second_prj_setup LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(NLOPT REQUIRED nlopt)
find_package(PCL 1.3 REQUIRED)

add_executable(sample_consensus main.cpp ColorUtilities.h
        racingline_nlopt.cpp
        DistinguishLeftRight.cpp)
target_link_libraries(sample_consensus ${PCL_LIBRARIES})
target_include_directories(sample_consensus PRIVATE ${NLOPT_INCLUDE_DIRS} )
target_link_directories(sample_consensus PRIVATE ${NLOPT_LIBRARY_DIRS})
add_definitions(${NLOPT_CFLAGS_OTHER})

target_link_libraries(sample_consensus ${NLOPT_LIBRARIES})
