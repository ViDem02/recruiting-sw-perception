cmake_minimum_required(VERSION 3.5)

project(pcl_visualizer)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find the QtWidgets library
find_package(Qt6 QUIET COMPONENTS Concurrent OpenGL Widgets)
if (NOT Qt6_FOUND)
  find_package(Qt5 COMPONENTS Concurrent OpenGL Widgets REQUIRED)
  set(QTX Qt5)
else()
  set(QTX Qt6)
endif()

find_package(VTK REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(NLOPT REQUIRED nlopt)
find_package(PCL 1.7.1 REQUIRED)

# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

set(project_SOURCES main.cpp pclviewer.cpp
        logic/get_cones_cloud.h
        logic/cones_recog.cpp
        logic/cones_recog.h
        logic/distinguish_left_right.cpp
        logic/distinguish_left_right.h
        logic/get_cones_cloud.cpp
        logic/get_cones_cloud.h
        logic/get_obstacle_cloud.cpp
        logic/get_obstacle_cloud.h
        logic/racingline_nlopt.cpp
        logic/racingline_nlopt.h
)

add_executable(${PROJECT_NAME} ${project_SOURCES})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${QTX}::Widgets)
target_include_directories(${PROJECT_NAME} PRIVATE ${NLOPT_INCLUDE_DIRS} )
target_link_directories(${PROJECT_NAME} PRIVATE ${NLOPT_LIBRARY_DIRS})
add_definitions(${NLOPT_CFLAGS_OTHER})
target_link_libraries(${PROJECT_NAME} ${NLOPT_LIBRARIES})