cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)

include_directories(include)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(hello_world SHARED tutorial/hello_world.cc)
target_link_libraries(hello_world ${GAZEBO_LIBRARIES})

add_library(model_push SHARED tutorial/model_push.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES})

add_library(dynobjectgazebo SHARED src/dynamic_object_gazebo_interface.cpp)
target_link_libraries(dynobjectgazebo ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_library(dynobjectros SHARED src/dynamic_object_ros_interface.cpp)
target_link_libraries(dynobjectros ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_library(grassrow SHARED src/visual_grass_row.cpp)
target_link_libraries(grassrow ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_executable(personvel_pub src/grdyn_pub.cpp)
#include(FindBoost)
#  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
#  target_link_libraries(personvel_pub ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(personvel_pub ${GAZEBO_LIBRARIES})
