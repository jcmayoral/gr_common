cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


add_library(hello_world SHARED hello_world.cc)
target_link_libraries(hello_world ${GAZEBO_LIBRARIES})

add_library(model_push SHARED model_push.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES})

add_library(grdyn_plugin SHARED gr_dyn_obstacle_plugin.cc)
target_link_libraries(grdyn_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_library(grdyn_plugin2 SHARED gr_dyn_obstacle_plugin_2.cc)
target_link_libraries(grdyn_plugin2 ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_executable(personvel_pub grdyn_pub.cc)
#include(FindBoost)
#  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
#  target_link_libraries(personvel_pub ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(personvel_pub ${GAZEBO_LIBRARIES})
