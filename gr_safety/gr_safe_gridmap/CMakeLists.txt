cmake_minimum_required(VERSION 2.8.3)
project(gr_safe_gridmap)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  angles
  grid_map_core
  grid_map_ros
  #grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization
  dynamic_reconfigure
  #ddynamic_reconfigure
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
find_package(YAML-CPP REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/SafeGridMap.cfg
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
	safety_msgs 
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
  src/safe_gridmap.cpp
)

add_executable(
  safe_gridmap_tutorial
  src/safe_gridmap_tutorial.cpp
)


add_executable(
  safe_gridmap_node
  src/safe_gridmap_node.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(
  safe_gridmap_tutorial
  ${OCTOMAP_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES} 
)


target_link_libraries(
  safe_gridmap_node
  #${PROJECT_NAME}
  ${OCTOMAP_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES} 
  #{YAML_CPP_LIBRARIES} 
  yaml-cpp
)

add_dependencies(safe_gridmap_node ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# Mark executables and/or libraries for installation
install(
  TARGETS safe_gridmap_tutorial safe_gridmap_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg)
