cmake_minimum_required(VERSION 3.23)

project(dcoss CXX)
set(CMAKE_CXX_STANDARD 17)

# ============ OpenMP ============
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP found, adding flags: ${OpenMP_CXX_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# ============ Gurobi ============
set(GUROBI_INCLUDE_DIR "/opt/gurobi1200/linux64/include")
set(GUROBI_LIBRARY "/opt/gurobi1200/linux64/lib/libgurobi120.so")
set(GUROBI_CXX_LIBRARY "/opt/gurobi1200/linux64/lib/libgurobi_c++.a")

include_directories(${GUROBI_INCLUDE_DIR})

# ============ Sources ============
set(sources
        src/core/algorithms.cpp
        src/core/algorithms.h
        src/io/input.cpp
        src/io/input.h
        src/io/output.cpp
        src/io/output.h
        src/core/deployment.cpp
        src/core/deployment.h
        src/util/util.cpp
        src/util/util.h
        src/main.cpp
)

add_executable(${PROJECT_NAME} ${sources})

# ============ Linking ============
target_link_libraries(${PROJECT_NAME}
        PRIVATE ${GUROBI_CXX_LIBRARY}
        PRIVATE ${GUROBI_LIBRARY}
        PUBLIC OpenMP::OpenMP_CXX
)

# ============ Build Type ============
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
