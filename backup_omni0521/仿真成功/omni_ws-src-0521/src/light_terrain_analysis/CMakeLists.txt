cmake_minimum_required(VERSION 3.0.2)
project(light_terrain_analysis)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  cv_bridge
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs tf2_ros tf2_geometry_msgs pcl_ros pcl_conversions cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(light_terrain_analysis_node src/light_terrain_analysis_node.cpp)
target_link_libraries(light_terrain_analysis_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS light_terrain_analysis_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
