cmake_minimum_required(VERSION 3.8)
project(lidars1)

# [通用配置]
find_package(OpenCV REQUIRED COMPONENTS
  core
  highgui
  imgproc
  imgcodecs
)

# [ROS2专用配置]
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# 包含目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  image_transport
  sensor_msgs
  cv_bridge
)
ament_export_include_directories(include)

# [构建目标]
add_executable(lidars1
  src/tcp_image_client.cpp
)

# 链接库
target_link_libraries(lidars1
  ${OpenCV_LIBS}
  image_transport::image_transport
  cv_bridge::cv_bridge
  rclcpp::rclcpp
)

# [安装配置]===============================================
# 关键修复：将可执行文件安装到 lib/${PROJECT_NAME}
install(TARGETS lidars1
  DESTINATION lib/${PROJECT_NAME}  # 修改为正确路径
)

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()