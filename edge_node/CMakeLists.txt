cmake_minimum_required(VERSION 3.0.2)
project(edge_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  actionlib_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  omni_stitch_capture
  pcl_ros
  tf
  cv_bridge
  image_transport
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem date_time iostreams regex)

find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp)
if(NOT NLOHMANN_JSON_INCLUDE_DIR)
  message(FATAL_ERROR "nlohmann/json.hpp not found. Please install nlohmann-json3-dev.")
endif()

find_library(LZ4_LIBRARY NAMES lz4 REQUIRED)
find_library(CURL_LIBRARY NAMES curl REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    actionlib_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    omni_stitch_capture
    pcl_ros
    tf
    cv_bridge
    image_transport
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${NLOHMANN_JSON_INCLUDE_DIR}
)

add_executable(edge_agent_node
  src/edge_agent_node.cpp
)

add_dependencies(edge_agent_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(edge_agent_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${LZ4_LIBRARY}
  ${CURL_LIBRARY}
)
