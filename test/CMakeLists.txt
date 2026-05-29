cmake_minimum_required(VERSION 3.10)
project(pipeline_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -g -pthread")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find PCL
find_package(PCL REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Find yaml-cpp (needed by patchworkpp include chain)
find_package(yaml-cpp REQUIRED)

# Include directories (relative to project root)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

# Link directories
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Pipeline test executable
add_executable(pipeline_test
    pipeline_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/utils/patchworkpp.cpp
)

target_link_libraries(pipeline_test
    ${PCL_LIBRARIES}
    yaml-cpp
)

# Convenience: copy output to a known location
set_target_properties(pipeline_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build
)
