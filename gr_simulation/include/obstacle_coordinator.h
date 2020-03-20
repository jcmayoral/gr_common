#ifndef _GAZEBO_OBSTACLE_COORDINATOR_HH_
#define _GAZEBO_OBSTACLE_COORDINATOR_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <gazebo/gazebo_config.h>
#include <gazebo/gazebo_client.hh>
#include <motion_planner.h>

namespace gazebo{
  class ObstaclesCoordinator{
    public:
      ObstaclesCoordinator(int nobstacles, std::string rootname="my_person");
     ~ObstaclesCoordinator();

   private:
    std::vector<MotionPlanner> obstacles_motion_planners_;
    transport::NodePtr node_;
  };
}

#endif