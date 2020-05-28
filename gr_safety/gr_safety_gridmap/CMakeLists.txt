cmake_minimum_required(VERSION 2.8.3)
project(gr_safety_gridmap)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_core
  grid_map_ros
  #grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  safety_msgs
  cv_bridge
  octomap_msgs
  filters
  topic_tools
  tf2
  tf2_ros
)

find_package(OpenCV 3 REQUIRED)
find_package(octomap REQUIRED)
find_package(YAML-CPP 0.6 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(OpenCV_INCLUDE_DIRS "${OpenCV_INSTALL_PATH}/include/opencv;${OpenCV_INSTALL_PATH}/include")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
)

add_library(
  ${PROJECT_NAME}
  src/safety_gridmap.cpp
)

add_executable(
  safety_gridmap_tutorial
  src/safety_gridmap_tutorial.cpp
)


add_executable(
  safety_gridmap_node
  src/safety_gridmap_node.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  safety_gridmap_tutorial
  ${OCTOMAP_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES} 
)

target_link_libraries(
  safety_gridmap_node
  ${PROJECT_NAME}
  ${OCTOMAP_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES} 
  yaml-cpp
)


# Mark executables and/or libraries for installation
install(
  TARGETS safety_gridmap_tutorial safety_gridmap_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
