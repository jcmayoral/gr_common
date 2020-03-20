#include <obstacle_coordinator.h>

using namespace gazebo;

ObstaclesCoordinator::ObstaclesCoordinator(int nobstacles, std::string rootname){
    node_ = transport::NodePtr(new transport::Node());
    node_->Init("obstacle_coordinator");
    std::cout << "initializing " << nobstacles << std::endl;
    std::string obstacleid;
    for (auto i=0; i < nobstacles; i++){
        
        if (i==0){
            obstacleid = "my_person";
        }
        else{
            obstacleid = "my_person_"+std::to_string(i-1);
        }
        std::cout << "sending " << obstacleid << std::endl;
        obstacles_motion_planners_.emplace_back(node_, obstacleid);
    }
}

ObstaclesCoordinator::~ObstaclesCoordinator(){

}