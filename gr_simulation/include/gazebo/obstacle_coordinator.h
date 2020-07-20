#ifndef _GAZEBO_OBSTACLE_COORDINATOR_HH_
#define _GAZEBO_OBSTACLE_COORDINATOR_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <gazebo/gazebo_config.h>
#include <gazebo/gazebo_client.hh>
#include <gazebo/motion_planner.h>
#include <thread>

namespace gazebo{
  class ObstaclesCoordinator{
    public:
      ObstaclesCoordinator(int nobstacles, double mapsize=40.0,  std::string rootname="my_person");
     ~ObstaclesCoordinator();
     void start();

   private:
    std::vector<std::thread> obstacles_motion_planners_;
    transport::NodePtr node_;
    std::condition_variable* cv_;
  };
}

#endif