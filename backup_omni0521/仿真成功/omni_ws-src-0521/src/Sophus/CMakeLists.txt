cmake_minimum_required(VERSION 3.0.2)
project(sophus)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  add_definitions("-Wall -Werror -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
endif()

find_package(catkin REQUIRED COMPONENTS cmake_modules)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS .
  LIBRARIES Sophus
  DEPENDS EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

set(SOURCE_DIR "sophus")
set(CLASSES
  so2
  se2
  se3
  so3
  scso3
  sim3
)

set(SOURCES)
foreach(class ${CLASSES})
  list(APPEND SOURCES "${SOURCE_DIR}/${class}.cpp" "${SOURCE_DIR}/${class}.h")
endforeach()

add_library(Sophus SHARED ${SOURCES})
target_link_libraries(Sophus ${catkin_LIBRARIES})

install(
  DIRECTORY sophus
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(
  TARGETS Sophus
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)