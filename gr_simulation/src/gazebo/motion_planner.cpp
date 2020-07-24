#include <gazebo/motion_planner.h>

using namespace gazebo;

MotionPlanner::MotionPlanner(std::string primfile): primitives_filename_(primfile),obstacleid_("defaults"), initialized_(false),ncells_(100), mtx_(), resolution_(0.1){
  std::cout << "constructor";
}

bool MotionPlanner::operator()(gazebo::transport::NodePtr node, std::string obstacleid, double mapsize, const msgs::Vector3d* goal){
  std::cout << "OPERATOR "<<std::endl;
  if (goal == NULL){
    current_goal_.set_x(1.0);
    current_goal_.set_y(1.0);
    current_goal_.set_z(3.1415);
  }

  else{
    current_goal_ = *goal;
  }
  
  return run(node, obstacleid, mapsize);
}

void MotionPlanner::setupMap(std::string obstacleid, double mapsize){
  obstacleid_ = obstacleid;
  double map_size = mapsize; //meters
  std::cout << "Creating Motion model for  " << obstacleid << std::endl;
  std::cout << "Map Size  " << mapsize << std::endl;
  map_size_ = map_size;

  env_ = new EnvironmentNAVXYTHETALAT();

  planner_ = new ARAPlanner(env_, false); //forward_search
  //TODO PARAMETRIZE

  //resolution_ = 0.1;
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
  //std::string primitive_filename_;
  //primitive_filename_ = "my_mprim_test.mprim";

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
                              primitives_filename_.c_str());
  }
  catch(SBPL_Exception e){
    std::cerr << e.what() << std::endl;
    std::cerr << "SBPL encountered a fatal exception!"<< std::endl;
    ret = false;
  }
}

bool MotionPlanner::run(gazebo::transport::NodePtr node, std::string obstacleid, double mapsize){
  setupMap(obstacleid, mapsize);
  vel_pub_ = node->Advertise<gazebo::msgs::Vector3d>("/" + obstacleid + "/vel_cmd");
  vel_pub_->WaitForConnection();
  odom_sub_ = node->Subscribe("/" + obstacleid + "/odom",&MotionPlanner::OnMsg, this);
  return performMotion();
}

bool MotionPlanner::performMotion(){
  std::cout << "performing motion "<< std::endl;

  double motionx;
  double motiony;

  //motionx = 0.10;//(map_size_) *(rand()/(double)RAND_MAX) - map_size_/2;
  //motiony = 0.0;//(map_size_) *(rand()/(double)RAND_MAX) - map_size_/2;
  //std::cout << "GOAL " << motionx << " , " << motiony<< std::endl;

  auto plan_result = planPath();
  std::cout << "Planned " << plan_result << " for " << obstacleid_ <<std::endl;
  std::cout << "Plan size " << sbpl_path_.size() << std::endl;
  error_ = -1;
  double dist2Goal = 10000;

  if (!plan_result){
    std::cout << "failuire on path planning motion " << std::endl;
    return false;
  }

  while(sbpl_path_.size()>1 && dist2Goal> 0.25){
    ExecuteCommand();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    dist2Goal = sqrt(pow(current_goal_.x() - current_pose_.position().x() ,2) + pow(current_goal_.y() - current_pose_.position().y() ,2));
    std::cout << dist2Goal << " : " << obstacleid_ << "and " << sbpl_path_.size() << ": " << copy_sbpl_path_.size() << std::endl;
  }
  stop();

  std::cout << "END motion number person id:  " << obstacleid_ << std::endl;
  return true;
  //std::cout << " x " << current_pose_.x() << " , " << current_pose_.y() << std::endl;

  //going backwards
}

void MotionPlanner::ExecuteCommand(){
    //mtx_.lock();
    boost::mutex::scoped_lock lock(mtx_);
    auto expected_pose = sbpl_path_.back();
    sbpl_path_.pop_back();
    
    //assert(velx < 2.0);
    //assert(vely < 2.0)
    auto currentyaw = msgs::ConvertIgn(current_pose_.orientation()).Yaw();
    auto angacc = (expected_pose.theta - currentyaw);

    std::cout << "expected pose " << (expected_pose.x - offset_)  << " : " << (expected_pose.y - offset_) << " :" << expected_pose.theta << std::endl; 
    std::cout << "current pose " << (current_pose_.position().x())  << " : " << (current_pose_.position().y()) << " : " << currentyaw << std::endl;
    auto auxx = (expected_pose.x - offset_) - (current_pose_.position().x());
    auto auxy = (expected_pose.y - offset_) - (current_pose_.position().y());

    auto velx = auxx*cos(currentyaw) - auxy*sin(currentyaw);
    auto vely = auxx*sin(currentyaw) + auxy*cos(currentyaw);

    error_ += sqrt(pow(velx,2) + pow(vely,2));

    msgs::Vector3d msg;    
    gazebo::msgs::Set(&msg, ignition::math::Vector3d(velx*0.1,vely*0.1,angacc*0.1));
    //std::cout << "velx " << velx << " vely " << vely << " ang acc " << angacc << "YAW " << currentyaw << "offset " << offset_ << std::endl;
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
    //std::lock_guard<std::mutex> guard(mtx_);
    current_pose_ = *_msg;//->position();
}

bool MotionPlanner::planPath(){
    while(!initialized_){
    }
    sbpl_path_.clear();
    offset_ = ncells_/2 * resolution_;

  try{
    //Check conversion offset of map start with frame -> gr_map_utils
    //Substract offset
    auto startyaw = msgs::ConvertIgn(current_pose_.orientation()).Yaw();
    startpose_ = current_pose_;
    int ret = env_->SetStart(current_pose_.position().x()+offset_, current_pose_.position().y()+offset_, startyaw);

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
    //current_goal_.set_x(goalx);
    //current_goal_.set_y(goaly);
    int ret = env_->SetGoal(current_goal_.x()+offset_, current_goal_.y()+offset_, current_goal_.z());

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
        current_pose_.position().x(),
        current_pose_.position().y(),
        0);
    sbpl_path_.push_back(s);
  }

  std::reverse(sbpl_path_.begin(), sbpl_path_.end());

  copy_sbpl_path_ = sbpl_path_;
  //sbpl_path[i].y;
  //sbpl_path[i].theta);
  return true;
}
