cmake_minimum_required(VERSION 3.28)
project(4_modules LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(4_modules main.cpp)

target_sources(4_modules
        PRIVATE
        FILE_SET cxx_modules TYPE CXX_MODULES FILES
        hello.ixx
)
