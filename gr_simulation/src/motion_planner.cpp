#include <motion_planner.h>

using namespace gazebo;

MotionPlanner::MotionPlanner(gazebo::transport::NodePtr node, std::string obstacleid): obstacleid_(obstacleid){
    odom_sub_ = node->Subscribe("/" + obstacleid + "/odom",&MotionPlanner::OnMsg, this);
}

void MotionPlanner::OnMsg(ConstPosePtr &_msg){
    std::cout << "odometry received on "<< odom_sub_->GetTopic() << std::endl;
}