#include <motion_planner.h>

using namespace gazebo;

MotionPlanner::MotionPlanner(): obstacleid_("defaults"){
}

void MotionPlanner::operator()(gazebo::transport::NodePtr node, std::string obstacleid){
    std::cout << "Creating Motion model for  " << obstacleid << std::endl;
    env_ = new EnvironmentNAVXYTHETALAT();

    planner_ = new ARAPlanner(env_, true); //forward_search
    
    //TODO PARAMETRIZE
    /*
    if(!env_->SetEnvParameter("cost_inscribed_thresh",costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))){
      ROS_ERROR("Failed to set cost_inscribed_thresh parameter");
      exit(1);
    }

    unsigned char cost_possibly_circumscribed_tresh = 0;
    if(!env_->SetEnvParameter("cost_possibly_circumscribed_thresh", costMapCostToSBPLCost(cost_possibly_circumscribed_tresh))){
      ROS_ERROR("Failed to set cost_possibly_circumscribed_thresh parameter");
      exit(1);
    }
    */

    double resolution = 1.0;
    double nominalvel_mpersecs = 0.4;
    double timetoturn45degsinplace_secs = 0.6;
    int obst_cost_thresh= 0;

    //circular footprint
    std::vector<sbpl_2Dpt_t> perimeterptsV;
    perimeterptsV.reserve(1);//circle;
    sbpl_2Dpt_t pt;
    perimeterptsV.push_back(pt);

    bool ret;
    std::string primitive_filename_;
    primitive_filename_ = "pr2.mprim";

    try{
      ret = env_->InitializeEnv(100,
                                100,
                                0, // mapdata
                                0, 0, 0, // start (x, y, theta, t)
                                0, 0, 0, // goal (x, y, theta)
                                0, 0, 0, //goal tolerance
                                perimeterptsV, resolution, nominalvel_mpersecs,
                                timetoturn45degsinplace_secs, obst_cost_thresh,
                                primitive_filename_.c_str());
    }
    catch(SBPL_Exception e){
      std::cerr << e.what() << std::endl;
      std::cerr << "SBPL encountered a fatal exception!"<< std::endl;
      ret = false;
    }

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