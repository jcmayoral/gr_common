#include <ros/motion_planner.h>

using namespace gazebo;

ROSMotionPlanner::ROSMotionPlanner(std::string primfile): primitives_filename_(primfile),obstacleid_("defaults"), initialized_(false),
                                                          nh("~"), ncells_(100), mtx_(), resolution_(0.1), map_set(false),odom_sub_(NULL){
  std::cout << "constructor";
}

bool ROSMotionPlanner::operator()(gazebo::transport::NodePtr node, std::string obstacleid, double mapsize, const msgs::Vector3d* goal){
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

void ROSMotionPlanner::setupMap(std::string obstacleid, double mapsize){
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

  map_set = true;
}

bool ROSMotionPlanner::run(gazebo::transport::NodePtr node, std::string obstacleid, double mapsize){
  if (!map_set){
    setupMap(obstacleid, mapsize);
    vel_pub_ = node->Advertise<gazebo::msgs::Vector3d>("/" + obstacleid + "/vel_cmd");
    vel_pub_->WaitForConnection();
    odom_sub_ = node->Subscribe("/" + obstacleid + "/odom",&ROSMotionPlanner::OnMsg, this);
    this->rpub_ = nh.advertise<visualization_msgs::Marker>( "/" + obstacleid + "/position", 1);
    this->rpub2_ = nh.advertise<nav_msgs::Path>( "/" + obstacleid + "/path", 1);
  }
  return performMotion();
}

void ROSMotionPlanner::publishPath(){
    nav_msgs::Path gui_path;
    gui_path.poses.resize(copy_sbpl_path_.size());
    gui_path.header.frame_id = "velodyne";//costmap_ros_->getGlobalFrameID();
    gui_path.header.stamp = ros::Time::now();
    std::cout << "SIZE OF PAth" << gui_path.poses.size() << std::endl;
    for(unsigned int i=0; i< copy_sbpl_path_.size(); i++){
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "velodyne";//costmap_ros_->getGlobalFrameID();
        pose.pose.position.x = copy_sbpl_path_[i].x - offset_;//offset.x;// + map_metadata_->origin.position.x;
        pose.pose.position.y = copy_sbpl_path_[i].y - offset_;// offset.y;// + map_metadata_->origin.position.y;
        pose.pose.position.z = 0;//start.pose.position.z;
        tf2::Quaternion temp;
        temp.setRPY(0,0,copy_sbpl_path_[i].theta);
        pose.pose.orientation.x = temp.getX();
        pose.pose.orientation.y = temp.getY();
        pose.pose.orientation.z = temp.getZ();
        pose.pose.orientation.w = temp.getW();
        //plan_.push_back(pose);
        gui_path.poses[i] = pose;
    }
    this->rpub2_.publish(gui_path);
}

bool ROSMotionPlanner::performMotion(){
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

  while(sbpl_path_.size()>1 && dist2Goal> 0.05){
    ExecuteCommand();
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    dist2Goal = sqrt(pow(current_goal_.x() - current_pose_.position().x() ,2) + pow(current_goal_.y() - current_pose_.position().y() ,2));
    std::cout << dist2Goal << " : " << obstacleid_ << "and " << sbpl_path_.size() << ": " << copy_sbpl_path_.size() << std::endl;
  }
  stop();

  std::cout << "END motion number person id:  " << obstacleid_ << std::endl;
  return true;
  //std::cout << " x " << current_pose_.x() << " , " << current_pose_.y() << std::endl;

  //going backwards
}

void ROSMotionPlanner::ExecuteCommand(){
    //mtx_.lock();
    boost::mutex::scoped_lock lock(mtx_);
    auto expected_pose = sbpl_path_.back();
    sbpl_path_.pop_back();

    //assert(velx < 2.0);
    //assert(vely < 2.0)
    auto currentyaw = msgs::ConvertIgn(current_pose_.orientation()).Yaw();

    tf2::Quaternion temp;
    //temp.setRPY(0,0,expected_pose.theta+M_PI/2);
    if (expected_pose.theta >= M_PI*2){
      temp.setRPY(0,0,-(expected_pose.theta-M_PI*2));
    }
    else{
      temp.setRPY(0,0,expected_pose.theta);
    }
    auto angacc = (temp.getAngleShortestPath() - currentyaw);// + M_PI) % (2*M_PI) - M_PI;
    //auto angacc = (temp.getAngle() - currentyaw);// + M_PI) % (2*M_PI) - M_PI;
    //auto angacc = std::min(std::fabs(expected_pose.theta/(2*M_PI) - currentyaw), std::fabs(currentyaw - expected_pose.theta/(2*M_PI)));


    std::cout << "DIFF " << temp.getAngleShortestPath() << " , " << currentyaw << std::endl;

    /*
    if (angacc > M_PI){
      angacc =
    }
    */

    std::cout << "expected pose " << (expected_pose.x )  << " : " << (expected_pose.y) << " : " << expected_pose.theta << std::endl;
    std::cout << "current pose " << (current_pose_.position().x() + offset_)  << " : " << (current_pose_.position().y() + offset_) << " : " << currentyaw << std::endl;
    auto auxx = (expected_pose.x) - (current_pose_.position().x() + offset_);
    auto auxy = (expected_pose.y) - (current_pose_.position().y() + offset_);
    std::cout << "VX " << auxx << " VY " << auxy <<  " angacc "<< angacc <<   std::endl;

    auto velx = auxx*cos(angacc) - auxy*sin(currentyaw);
    auto vely = auxx*sin(angacc) + auxy*cos(currentyaw);
    error_ += sqrt(pow(velx,2) + pow(vely,2));

    msgs::Vector3d msg;
    gazebo::msgs::Set(&msg, ignition::math::Vector3d(auxx,auxy,angacc));
    //std::cout << "velx " << velx << " vely " << vely << " ang acc " << angacc << "YAW " << currentyaw << "offset " << offset_ << std::endl;
    // Send the message
    vel_pub_->Publish(msg);


    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::DELETE;
    rpub_.publish(marker);
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = current_pose_.position().x();
    marker.pose.position.y = current_pose_.position().y();
    marker.pose.position.z = 1;

    auto yaw = msgs::ConvertIgn(current_pose_.orientation()).Yaw();
    tf2::Quaternion t;
    //temp.setRPY(0,0,expected_pose.theta+M_PI/2);
    t.setRPY(0,0,yaw);
    marker.pose.orientation = tf2::toMsg(t);
    marker.scale.x = 1.0;
    marker.scale.y = 0.1;
    marker.scale.z = 0.5;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    rpub_.publish( marker );
    //mtx_.unlock();
}

void ROSMotionPlanner::stop(){
  msgs::Vector3d msg;
    gazebo::msgs::Set(&msg, ignition::math::Vector3d(0,0,0.0));
    // Send the message
    vel_pub_->Publish(msg);
}


void ROSMotionPlanner::OnMsg(ConstPosePtr &_msg){
    //boost::mutex::scoped_lock lck(mtx_);
    initialized_ = true;
    //std::lock_guard<std::mutex> guard(mtx_);
    current_pose_ = *_msg;//->position();
}

bool ROSMotionPlanner::planPath(){
    while(!initialized_){
    }
    sbpl_path_.clear();
    offset_ = ncells_/2 * resolution_;
    std::cout << " OFFSET " << offset_ << std::endl;

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
  this->publishPath();
  return true;
}
