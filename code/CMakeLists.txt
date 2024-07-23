cmake_minimum_required(VERSION 3.28)
project(code)

set(CMAKE_CXX_STANDARD 14)

include_directories(src)

add_executable(code
        src/algorithms.cpp
        src/algorithms.h
        src/input.cpp
        src/input.h
        src/main.cpp
        src/output.cpp
        src/output.h
        src/deployment.h
        src/deployment.cpp)
