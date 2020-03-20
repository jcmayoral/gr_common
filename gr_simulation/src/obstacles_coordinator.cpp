#include <obstacle_coordinator.h>

using namespace gazebo;

ObstaclesCoordinator::ObstaclesCoordinator(int nobstacles, std::string rootname): node_(new gazebo::transport::Node()){
    node_->Init();
    //node_ = transport::NodePtr(new transport::Node());
    //node_->Init();
    std::cout << "initializing " << nobstacles << std::endl;
    std::string obstacleid;
    for (auto i=0; i < nobstacles; i++){

        if (i==0){
            obstacleid = "my_person";
        }
        else{
            obstacleid = "my_person_"+std::to_string(i-1);
        }
        obstacles_motion_planners_.emplace_back(std::thread(MotionPlanner(),node_, obstacleid));
    }
}

ObstaclesCoordinator::~ObstaclesCoordinator(){

}


void ObstaclesCoordinator::start(){
    for (auto it = obstacles_motion_planners_.begin(); it!= obstacles_motion_planners_.end(); it++){
        it->detach();
    }
    std::cout << "ending start";
}