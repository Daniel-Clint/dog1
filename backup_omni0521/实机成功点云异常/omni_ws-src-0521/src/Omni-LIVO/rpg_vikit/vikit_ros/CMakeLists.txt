SET(PROJECT_NAME vikit_ros)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)

# Set build flags
SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g")
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

# Add catkin and required ROS packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp vikit_common visualization_msgs tf cmake_modules sophus)

# Add plain cmake packages 
# Avoid OpenCV ABI mismatch with ROS Noetic's cv_bridge (OpenCV 4.2 on Ubuntu 20.04).
if(EXISTS "/usr/lib/x86_64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake")
  set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")
endif()
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Describe catkin Project
catkin_package(
    DEPENDS Eigen OpenCV vikit_common
    CATKIN_DEPENDS roscpp visualization_msgs tf sophus
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

# Set Sourcefiles
LIST(APPEND SOURCEFILES 
  src/output_helper.cpp
)

# Create vikit library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES})
