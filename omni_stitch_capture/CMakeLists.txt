cmake_minimum_required(VERSION 3.0.2)
project(omni_stitch_capture)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  message_filters
  std_msgs
)

add_message_files(
  FILES
  StitchedCapture.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    rospy
    sensor_msgs
    geometry_msgs
    nav_msgs
    cv_bridge
    message_filters
    std_msgs
)

catkin_install_python(PROGRAMS
  scripts/omni_stitch_capture_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
