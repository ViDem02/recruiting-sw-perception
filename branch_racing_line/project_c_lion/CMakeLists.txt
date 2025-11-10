cmake_minimum_required(VERSION 3.14)
project(racingline_nlopt_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(NLOPT REQUIRED nlopt)

# Find Point Cloud Library
find_package(PCL REQUIRED)
# TODO add other dependencies if needed

add_executable(racingline_nlopt racingline_nlopt.cpp)

target_include_directories(racingline_nlopt PRIVATE ${NLOPT_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_directories(racingline_nlopt PRIVATE ${NLOPT_LIBRARY_DIRS})
add_definitions(${NLOPT_CFLAGS_OTHER} ${PCL_DEFINITIONS})

target_link_libraries(racingline_nlopt ${NLOPT_LIBRARIES} ${PCL_LIBRARIES})


