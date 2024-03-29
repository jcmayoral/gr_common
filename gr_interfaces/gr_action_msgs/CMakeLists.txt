cmake_minimum_required(VERSION 3.0.2)
project(gr_action_msgs)

find_package(catkin REQUIRED
        COMPONENTS
            std_msgs
            geometry_msgs
	    safety_msgs
	    sensor_msgs
            actionlib_msgs
	    message_generation
	    darknet_ros_msgs
)


add_action_files(
  DIRECTORY action
  FILES GRNavigation.action SimMotionPlanner.action 
  	SingleRowExecution.action GRPCProcess.action
	GRDepthProcess.action
)

add_service_files(
  FILES
  GetMetrics.srv
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs 
  	geometry_msgs sensor_msgs
	safety_msgs darknet_ros_msgs
)


catkin_package(
    CATKIN_DEPENDS
        std_msgs
        geometry_msgs
	sensor_msgs
	safety_msgs
        actionlib_msgs
	darknet_ros_msgs
)


## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
