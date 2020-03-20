#include <motion_planner.h>

using namespace gazebo;

MotionPlanner::MotionPlanner(): obstacleid_("defaults"){
}

void MotionPlanner::operator()(gazebo::transport::NodePtr node, std::string obstacleid){
    std::cout << "here " << obstacleid << std::endl;
    vel_pub_ = node->Advertise<gazebo::msgs::Vector3d>("/" + obstacleid + "/vel_cmd");
    vel_pub_->WaitForConnection();
    odom_sub_ = node->Subscribe("/" + obstacleid + "/odom",&MotionPlanner::OnMsg, this);
    std::cout << odom_sub_->GetTopic() << std::endl;
    while(true);
} 

void MotionPlanner::OnMsg(ConstPosePtr &_msg){
    std::cout << "odometry received on "<< odom_sub_->GetTopic() << std::endl;
    // Create a a vector3 message
    msgs::Vector3d msg;
    // Set the velocity in the x-component
    gazebo::msgs::Set(&msg, ignition::math::Vector3d(0,0,1.0));
    // Send the message
    vel_pub_->Publish(msg);
}