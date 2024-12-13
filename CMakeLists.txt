cmake_minimum_required(VERSION 3.23)
include(FetchContent)

FetchContent_Declare(matplotplusplus
        GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
        GIT_TAG origin/master
)

FetchContent_GetProperties(matplotplusplus)

if (NOT matplotplusplus_POPULATED)
    FetchContent_MakeAvailable(matplotplusplus)
endif ()

project(dcoss)

set(CMAKE_CXX_STANDARD 17)

option(CXX "enable C++ compilation" ON)

if (CXX)
    enable_language(CXX)
endif ()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Visual Studio compiler with static runtime libraries
if (MSVC AND MT)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MTd")
endif ()

find_package(GUROBI REQUIRED)

include_directories(${GUROBI_INCLUDE_DIRS})

# list source files here
set(sources src/core/algorithms.cpp
        src/core/algorithms.h
        src/io/input.cpp
        src/io/input.h
        src/io/output.cpp
        src/io/output.h
        src/core/deployment.cpp
        src/core/deployment.h
        src/util/util.cpp
        src/util/util.h
        src/io/plots.cpp
        src/io/plots.h
        src/main.cpp
)

add_executable(${CMAKE_PROJECT_NAME} ${sources})

if (CXX)
    set(CMAKE_CXX_STANDARD 17)
    target_link_libraries(${CMAKE_PROJECT_NAME} optimized ${GUROBI_CXX_LIBRARY}
            debug ${GUROBI_CXX_DEBUG_LIBRARY})
endif ()

target_link_libraries(${CMAKE_PROJECT_NAME} ${GUROBI_LIBRARY} matplot)

if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    include(FeatureSummary)
    feature_summary(WHAT ALL)
endif ()
