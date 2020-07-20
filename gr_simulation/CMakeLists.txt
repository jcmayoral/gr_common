cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_compile_options(-std=c++17)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
find_library(SBPL_LIBRARY NAMES sbpl PATHS ${SBPL_LIBRARY_DIRS} NO_DEFAULT_PATH)

include_directories(include
                    ${roscpp_INCLUDE_DIRS}
                    ${geometry_msgs_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_BINARY_DIR}/msgs
                    ${SBPL_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS} 
                 ${CMAKE_CURRENT_BINARY_DIR}/msgs
                 ${SBPL_LIBRARY_DIRS})
add_subdirectory(msgs)

#link_directories(${GAZEBO_LIBRARY_DIRS})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(dynobjectgazebo SHARED src/gazebo/dynamic_object_gazebo_interface.cpp)
target_link_libraries(dynobjectgazebo ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_library(dynobjectros SHARED src/ros/dynamic_object_ros_interface.cpp)
target_link_libraries(dynobjectros ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_library(grassrow SHARED src/grassrobot/grass_row.cpp)
target_link_libraries(grassrow ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} gr_simulation_msgs)

add_library(visualgrassrow SHARED src/grassrobot/visual_grass_row.cpp)
target_link_libraries(visualgrassrow ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} gr_simulation_msgs)

add_library(motionplanner SHARED src/common/motion_planner.cpp)
target_link_libraries(motionplanner ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${SBPL_LIBRARY})

add_library(obstaclescoordinator SHARED src/common/obstacles_coordinator.cpp)
target_link_libraries(obstaclescoordinator ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} motionplanner)

add_executable(personvel_pub src/gazebo/grdyn_pub.cpp)
add_executable(grass_cutter src/grassrobot/grass_event_caller.cpp)
add_executable(run_obstacle_coordinator src/gazebo/obstacles_coordinator_node.cpp)
#include(FindBoost)
#  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
#  target_link_libraries(personvel_pub ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(personvel_pub ${GAZEBO_LIBRARIES})
target_link_libraries(grass_cutter ${GAZEBO_LIBRARIES} gr_simulation_msgs)
target_link_libraries(run_obstacle_coordinator ${GAZEBO_LIBRARIES} obstaclescoordinator)

file(GLOB PRIMITIVES_FILES
  "*.mprim"
)
file(COPY ${PRIMITIVES_FILES} 
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
