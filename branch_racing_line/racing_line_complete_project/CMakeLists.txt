cmake_minimum_required(VERSION 3.10)
project(racing_line_complete_project)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find PCL
find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ConesRecognizer library
add_library(cones_recognizer
    ConesRecognizer.cpp
    ConesRecognizer.h
)
target_link_libraries(cones_recognizer ${PCL_LIBRARIES})

# DistinguishLeftRight library
add_library(distinguish_left_right
    DistinguishLeftRight.cpp
    DistinguishLeftRight.h
)
target_link_libraries(distinguish_left_right ${PCL_LIBRARIES})

# Legacy cones_recog library (procedural version)
add_library(cones_recog
    cones_recog.cpp
    cones_recog.h
)
target_link_libraries(cones_recog ${PCL_LIBRARIES})

# Main executable using refactored class
add_executable(racingline_nlopt
    racingline_nlopt.cpp
)
target_link_libraries(racingline_nlopt
    cones_recognizer
    distinguish_left_right
    ${PCL_LIBRARIES}
)

# Legacy cones_recog library already built above - no separate executable needed
# (The procedural API is demonstrated through unit tests or can be linked into other executables)

# Test executable demonstrating the refactored API
add_executable(test_cones_recognizer
    test_cones_recognizer.cpp
)
target_link_libraries(test_cones_recognizer
    cones_recognizer
    distinguish_left_right
    ${PCL_LIBRARIES}
)
