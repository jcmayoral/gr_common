cmake_minimum_required(VERSION 3.0.2)
project(navigation_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
   std_msgs
   geometry_msgs
   message_generation)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   TopologicalMap.msg
   TopologicalNode.msg
   Vertex.msg
   Edge.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs 
   geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS 
    message_runtime
    std_msgs 
    geometry_msgs
)
