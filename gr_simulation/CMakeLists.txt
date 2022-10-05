cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gr_simulation)

add_compile_options(-std=c++17)

find_package(roscpp REQUIRED COMPONENTS std_msgs geometry_msgs gr_action_msgs nav_msgs visualization_msgs)
find_package(gr_action_msgs REQUIRED)
find_package(actionlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(safety_msgs REQUIRED)


find_package(gazebo REQUIRED)
find_package(tf2_ros REQUIRED COMPONENTS tf2_geometry_msgs)

find_package(dynamic_reconfigure REQUIRED)


find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
find_library(SBPL_LIBRARY NAMES sbpl PATHS ${SBPL_LIBRARY_DIRS} NO_DEFAULT_PATH)

generate_dynamic_reconfigure_options(
  cfg/PersonMotion.cfg
)

include_directories(include
                    ${roscpp_INCLUDE_DIRS}
                    ${geometry_msgs_INCLUDE_DIRS}
                    ${gr_action_msgs_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_BINARY_DIR}/msgs
                    ${SBPL_INCLUDE_DIRS}
                    ${safety_msgs_INCLUDE_DIRS}
                    )

link_directories(${Boost_INCLUDE_DIRS}
                 ${GAZEBO_LIBRARY_DIRS}
                 ${CMAKE_CURRENT_BINARY_DIR}/msgs
                 ${SBPL_LIBRARY_DIRS})
add_subdirectory(msgs)

#link_directories(${GAZEBO_LIBRARY_DIRS})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
#NO
add_library(dynobjectgazebo SHARED src/gazebo/dynamic_object_gazebo_interface.cpp)
target_link_libraries(dynobjectgazebo ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
#NO
add_library(grassrow SHARED src/grassrobot/grass_row.cpp)
target_link_libraries(grassrow ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} gr_simulation_msgs)
#NO
add_library(visualgrassrow SHARED src/grassrobot/visual_grass_row.cpp)
target_link_libraries(visualgrassrow ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} gr_simulation_msgs)

#NO
add_library(motionplanner SHARED src/gazebo/motion_planner.cpp)
target_link_libraries(motionplanner ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${SBPL_LIBRARY})

#NO
add_library(rosmotionplanner SHARED src/ros/motion_planner.cpp)
target_link_libraries(rosmotionplanner ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${SBPL_LIBRARY} ${tf2_ros_LIBRARIES})

#NO
add_library(obstaclescoordinator SHARED src/gazebo/obstacles_coordinator.cpp)
target_link_libraries(obstaclescoordinator ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} motionplanner)

#ROS
add_library(dynobjectros SHARED src/ros/dynamic_object_ros_interface.cpp)
target_link_libraries(dynobjectros ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${actionlib_LIBRARIES} rosmotionplanner)
#add_dependencies(dynobjectros   ${gr_action_msgs_EXPORTED_TARGETS})
#NO
add_library(myrosgazeboanimation SHARED src/ros/gazebo_ros_animation.cpp)
target_link_libraries(myrosgazeboanimation ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${actionlib_LIBRARIES} motionplanner)


add_library(simplerosinterface SHARED src/ros/simple_ros_interface.cpp)
target_link_libraries(simplerosinterface ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${dynamic_reconfigure_LIBRARIES} ${actionlib_LIBRARIES})
#add_dependencies(simplerosinterface ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(rosinterface SHARED src/ros/ros_interface.cpp)
target_link_libraries(rosinterface ${roscpp_LIBRARIES} ${dynamic_reconfigure_LIBRARIES} ${actionlib_LIBRARIES})
add_dependencies(rosinterface ${PROJECT_NAME}_gencfg)


add_executable(personvel_pub src/gazebo/grdyn_pub.cpp)
add_executable(grass_cutter src/grassrobot/grass_event_caller.cpp)
add_executable(run_obstacle_coordinator src/gazebo/obstacles_coordinator_node.cpp)
add_executable(ros_interface_node src/ros/ros_interface_node.cpp)

#include(FindBoost)
#  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
#  target_link_libraries(personvel_pub ${GAZEBO_IBRARIES} ${Boost_LIBRARIES})
target_link_libraries(personvel_pub ${GAZEBO_LIBRARIES})
target_link_libraries(grass_cutter ${GAZEBO_LIBRARIES} gr_simulation_msgs)
target_link_libraries(run_obstacle_coordinator ${GAZEBO_LIBRARIES} obstaclescoordinator)
target_link_libraries(ros_interface_node ${GAZEBO_LIBRARIES} rosinterface ${tf2_ros_LIBRARIES})

file(GLOB PRIMITIVES_FILES
  "*.mprim"
)
file(COPY ${PRIMITIVES_FILES}
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(GLOB WORLDS_FILES
  "worlds/*.world"
)
file(COPY ${WORLDS_FILES}
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

install(TARGETS ros_interface_node rosinterface
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION /home/jose/local/lib
  RUNTIME DESTINATION /home/jose/local/bin
  )
