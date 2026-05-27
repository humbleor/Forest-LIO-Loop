cmake_minimum_required(VERSION 3.10)
project(forest_loop_detector)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -g -pthread")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  std_msgs
  geometry_msgs
  visualization_msgs
  message_filters
  cmake_modules
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(small_gicp REQUIRED QUIET)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs pcl_ros pcl_conversions std_msgs geometry_msgs visualization_msgs message_filters
  DEPENDS PCL GTSAM OpenCV
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  ${small_gicp_INCLUDE_DIRS}
)

# Source files
file(GLOB utils_sources src/utils/*.cpp)
file(GLOB gtsam_opti_sources src/gtsam_opti/*.cpp)

# Main loop detector node
add_executable(loop_detector_node
  src/loop_detector_node.cpp
  src/KeyframeManager.cpp
  src/utils/HashRegObj.cpp
  src/utils/Hlp.cpp
  src/utils/patchworkpp.cpp
  ${gtsam_opti_sources}
)
add_dependencies(loop_detector_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(loop_detector_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  gtsam
  yaml-cpp
  small_gicp
  ${OpenMP_CXX_FLAGS}
)
