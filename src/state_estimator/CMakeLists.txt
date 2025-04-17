cmake_minimum_required(VERSION 3.0.2)
project(state_estimator)


set(EIGEN3_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/src/eigen")
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
## Find catkin and packages we need
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
)



## Catkin specific configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES state_estimator
  CATKIN_DEPENDS roscpp nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs message_filters
)

## Build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ executable
add_executable(state_estimator_start src/state_estimator.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(state_estimator_start
  ${catkin_LIBRARIES}
)

## Mark executables for installation
install(TARGETS state_estimator_start
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)