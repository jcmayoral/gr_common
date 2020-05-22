#include <motion_planner.h>

using namespace gazebo;

MotionPlanner::MotionPlanner(): obstacleid_("defaults"), initialized_(false),ncells_(100), mtx_(){
  std::cout << "constructor";
}


void MotionPlanner::operator()(gazebo::transport::NodePtr node, std::string obstacleid, double mapsize){
  run(node, obstacleid, mapsize);
}

bool MotionPlanner::run(gazebo::transport::NodePtr node, std::string obstacleid, double mapsize){
    obstacleid_ = obstacleid;
    double map_size = mapsize; //meters
    std::cout << "Creating Motion model for  " << obstacleid << std::endl;
    std::cout << "Map Size  " << mapsize << std::endl;

    env_ = new EnvironmentNAVXYTHETALAT();

    planner_ = new ARAPlanner(env_, false); //forward_search
    //TODO PARAMETRIZE

    resolution_ = 0.1;
    double nominalvel_mpersecs = 0.1;
    double timetoturn45degsinplace_secs = 0.1;
    int obst_cost_thresh= 2.0;

    if(!env_->SetEnvParameter("cost_inscribed_thresh",2.0)){
      exit(1);
    }

    unsigned char cost_possibly_circumscribed_tresh = 0;
    if(!env_->SetEnvParameter("cost_possibly_circumscribed_thresh", cost_possibly_circumscribed_tresh)){
      exit(1);
    }

    //circular footprint
    std::vector<sbpl_2Dpt_t> perimeterptsV;
    perimeterptsV.reserve(1);//circle;
    sbpl_2Dpt_t pt_m;
    double halfwidth = 0.01; //0.3;
    double halflength = 0.01; //0.45;
    pt_m.x = -halflength;
    pt_m.y = -halfwidth;
    perimeterptsV.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = -halfwidth;
    perimeterptsV.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = halfwidth;
    perimeterptsV.push_back(pt_m);
    pt_m.x = -halflength;
    pt_m.y = halfwidth;
    perimeterptsV.push_back(pt_m);

    bool ret;
    std::string primitive_filename_;
    primitive_filename_ = "my_mprim_test.mprim";

    ncells_ = int(map_size/resolution_ + 1);

    try{
      ret = env_->InitializeEnv(ncells_,
                                ncells_,
                                0, // mapdata
                                0, 0, 0, // start (x, y, theta, t)
                                0, 0, 0, // goal (x, y, theta)
                                0, 0, 0, //goal tolerance
                                perimeterptsV, resolution_, nominalvel_mpersecs,
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
    performMotion();
}

void MotionPlanner::performMotion(){

  double motionx;
  double motiony;

  auto repetitions = rand() % 10 + 1;

  for (auto i = 0; i<repetitions; i++){
    motionx = rand() % -10 + 10;
    motiony = rand() % -10 + 10;
    auto plan_result = planPath(motionx, motiony, 0);
    std::cout << "Planned " << plan_result << " for " << obstacleid_ <<std::endl;
    std::cout << "Plan size " << sbpl_path_.size() << std::endl;
    error_ = -1;
    double dist2Goal = 10000;

    if (!plan_result){
      std::cout << "failuire on path planning motion " << i << std::endl;
      continue;
    }

    while(sbpl_path_.size()>1 && dist2Goal> 0.25){
      ExecuteCommand();
      std::this_thread::sleep_for(std::chrono::milliseconds(80));
      dist2Goal = sqrt(pow(current_goal_.x() - current_pose_.x() ,2) + pow(current_goal_.y() - current_pose_.y() ,2)); 
      std::cout << dist2Goal << " : " << obstacleid_ << "and " << sbpl_path_.size() << std::endl;
    }
    stop();

    std::cout << "END motion number" << i+1 << " of " << repetitions << " person id:  " << obstacleid_ << std::endl;
    //std::cout << " x " << current_pose_.x() << " , " << current_pose_.y() << std::endl;

    //going backwards
  }
}

void MotionPlanner::ExecuteCommand(){
    //mtx_.lock();
    boost::mutex::scoped_lock lock(mtx_);
    auto expected_pose = sbpl_path_.back();
    sbpl_path_.pop_back();
    //std::cout << " x " << (expected_pose.x - offset_) << " , " << current_pose_.x() << std::endl;
    auto velx = (expected_pose.x - offset_) - current_pose_.x();
    auto vely = (expected_pose.y - offset_) - current_pose_.y();
    //pos error
    error_ += sqrt(pow(velx,2) + pow(vely,2));
    //std::cout << "vel x " << velx <<  " vel y " << vely << std::endl;
    msgs::Vector3d msg;
    //assert(velx < 2.0);
    //assert(vely < 2.0);
    gazebo::msgs::Set(&msg, ignition::math::Vector3d(velx,vely,0.0));
    //std::cout << "velx " << velx << " vely " << vely << std::endl;
    // Send the message
    vel_pub_->Publish(msg);
    //mtx_.unlock();
}

void MotionPlanner::stop(){
  msgs::Vector3d msg;
    gazebo::msgs::Set(&msg, ignition::math::Vector3d(0,0,0.0));
    // Send the message
    vel_pub_->Publish(msg);
}


void MotionPlanner::OnMsg(ConstPosePtr &_msg){
    //boost::mutex::scoped_lock lck(mtx_);
    initialized_ = true;
    //mutex crashes -> it is a thread not so sure how to do it here.
    //std::lock_guard<std::mutex> guard(mtx_);
    //std::cout << "odometry received on "<< odom_sub_->GetTopic() << std::endl;
    current_pose_ = _msg->position();
    // Create a a vector3 message
}

bool MotionPlanner::planPath(double goalx, double goaly, double goalyaw){
    while(!initialized_){
      //std::cout << "wait for initialization" << std::endl;
    }
    sbpl_path_.clear();

    double theta_start = 0.0;//2 * atan2(start.pose.orientation.z, start.pose.orientation.w);
    offset_ = ncells_/2 * resolution_;

  try{
    //Check conversion offset of map start with frame -> gr_map_utils
    //Substract offset
    int ret = env_->SetStart(current_pose_.x()+offset_, current_pose_.y()+offset_, theta_start);

    if(ret < 0 || planner_->set_start(ret) == 0){
      std::cerr<<"ERROR: failed to set start state\n"<<std::endl;
      return false;
    }
  }
  catch(SBPL_Exception *e){
    std::cerr<<e->what()<<std::endl;
    std::cerr<<"SBPL encountered a fatal exception while setting the start state"<<std::endl;
    return false;
  }

  try{
    //Substract offset
    current_goal_.set_x(current_pose_.x()+goalx);
    current_goal_.set_y(current_pose_.y()+goaly);
    
    int ret = env_->SetGoal(current_goal_.x()+offset_, current_goal_.y()+offset_, goalyaw);

    if(ret < 0 || planner_->set_goal(ret) == 0){
      std::cerr<<"ERROR: failed to set goal state\n" << std::endl;
      return false;
    }
  }
  catch(SBPL_Exception *e){
    std::cerr<<e->what()<<std::endl;
    std::cerr<<"SBPL encountered a fatal exception while setting the goal state"<<std::endl;
    return false;
  }


  /*
  //Try to load environment from test files
  for(unsigned int ix = 0; ix < ncells_; ix++) {
    for(unsigned int iy = 0; iy < ncells_; iy++) {
      //first case - off cell goes on
      env_->UpdateCost(ix,iy, (unsigned char) 0);//costmap_ros_->getCostmap()->getCost(ix,iy)));
    }
  }
  */


  //TODO parametrize
  double allocated_time_ = 1.0;
  double initial_epsilon_ = 3.0;
  //setting planner parameters
  //ROS_DEBUG("allocated:%f, init eps:%f\n",allocated_time_,initial_epsilon_);
  planner_->set_initialsolution_eps(initial_epsilon_);
  planner_->set_search_mode(true);

  std::cout << "[sbpl_lattice_planner] run planner for " << obstacleid_ << std::endl;
  std::vector<int> solution_stateIDs;
  int solution_cost;
  try{
    int ret = planner_->replan(allocated_time_, &solution_stateIDs, &solution_cost);
    if(ret)
      std::cout << "Solution is found\n" << std::endl;
    else{
      std::cerr << "Solution not found\n"<< std::endl;
      //publishStats tion_cost, 0, start, goal);
      return false;
    }
  }
  catch(SBPL_Exception *e){
    std::cerr<<"SBPL encountered a fatal exception while planning"<<std::endl;
    return false;
  }

  try{
    env_->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &sbpl_path_);
  }
  catch(SBPL_Exception *e){
    std::cerr<<"SBPL encountered a fatal exception while reconstructing the path"<<std::endl;
    return false;
  }
  // if the plan has zero points, add a single point to make move_base happy
  if( sbpl_path_.size() == 0 ) {
    //Substract offset
    EnvNAVXYTHETALAT3Dpt_t s(
        current_pose_.x(),
        current_pose_.y(),
        0);
    sbpl_path_.push_back(s);
  }

  std::reverse(sbpl_path_.begin(), sbpl_path_.end());

  //sbpl_path[i].y;
  //sbpl_path[i].theta);
  return true;
}
