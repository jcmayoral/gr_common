#include<ros/ros_interface.h>
using namespace gr_simulation;


ROSInterface::ROSInterface(){
    ac_client_ = boost::make_shared<actionlib::SimpleActionClient<gr_action_msgs::SimMotionPlannerAction>>("SimMotionPlanner/my_person",true);
    ROS_INFO("WAIT FOR SERVER");
    ac_client_->waitForServer();
    ROS_INFO("SERVER FOUND");
    dyn_server_cb_ = boost::bind(&ROSInterface::dyn_reconfigureCB, this, _1, _2);
    dyn_server_.setCallback(dyn_server_cb_);
    ros::spin();
}


ROSInterface::~ROSInterface(){
    
}


void ROSInterface::dyn_reconfigureCB(gr_simulation::PersonMotionConfig &config, uint32_t level){
    std::cout << "BEFORE " << std::endl;
    std::cout << "AFTER " << std::endl;
}