cmake_minimum_required(VERSION 3.0.2)
project(lidars1)

# 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  sensor_msgs
  cv_bridge
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    image_transport 
    sensor_msgs 
)

# OpenCV依赖
find_package(OpenCV REQUIRED)

# 添加包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_executable(lidars1 src/tcp_image_client.cpp)

target_link_libraries(lidars1
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

# 安装规则
install(TARGETS lidars1
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
