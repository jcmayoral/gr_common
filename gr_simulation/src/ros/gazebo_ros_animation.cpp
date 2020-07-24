/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <functional>
#include <ros/dynamic_object_ros_animation.h>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(GazeboROSAnimation)

/////////////////////////////////////////////////
GazeboROSAnimation::GazeboROSAnimation(): is_motionfinished(true), motion_type("stand"){
    this->velocity = 0.1;
    if (!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client");//,ros::init_options::NoSigintHandler);
        ros::spinOnce();
    }
    std::cout << "RUN"<<std::endl;;

}

/////////////////////////////////////////////////
void GazeboROSAnimation::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  
  // Read in the target weight
  if (_sdf->HasElement("target_weight"))
    this->targetWeight = _sdf->Get<double>("target_weight");
  else
    this->targetWeight = 1.15;

  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
  else
    this->obstacleWeight = 1.5;

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  // Add our own name to models we should ignore when avoiding obstacles.
  this->ignoreModels.push_back(this->actor->GetName());

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles"))
  {
    sdf::ElementPtr modelElem =
      _sdf->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem)
    {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }
  ros::NodeHandle nh;// = boost::make_shared<ros::NodeHandle>("~");
  nh.setCallbackQueue(&my_callback_queue);

  aserver = boost::make_shared<actionlib::SimpleActionServer<gr_action_msgs::SimMotionPlannerAction>>(nh, std::string("SimMotionPlanner")+"/" + this->model->GetName(), 
                                                                boost::bind(&GazeboROSAnimation::executeCB, this, _1), false);
  this->rosQueueThread = std::thread(std::bind(&GazeboROSAnimation::QueueThread, this));
  aserver->start();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&GazeboROSAnimation::OnUpdate, this, std::placeholders::_1)));

  this->Reset();
}


void GazeboROSAnimation::executeCB(const gr_action_msgs::SimMotionPlannerGoalConstPtr &goal){
  gr_action_msgs::SimMotionPlannerFeedback feedback;
  gr_action_msgs::SimMotionPlannerResult result;

  if (std::find(std::begin(AVAILABLEMOTIONS), std::end(AVAILABLEMOTIONS), goal->motion_type) == std::end(AVAILABLEMOTIONS)){
    ROS_ERROR("Motion request not found ");
    aserver->setAborted(result);
    return;
  }

  motion_type = static_cast<std::string> (goal->motion_type);

  ignition::math::Pose3d pose;
  if(goal->setstart){
    pose.Pos().X(goal->startpose.pose.position.x);
    pose.Pos().Y(goal->startpose.pose.position.y);
    pose.Pos().Z(goal->startpose.pose.position.z+1.05);
    this->model->SetWorldPose(pose);
    this->actor->SetWorldPose(pose, true, true);
    this->Reset();
  }

  ignition::math::Vector3d newTarget;
  newTarget.X(goal->goalPose.pose.position.x);
  newTarget.Y(goal->goalPose.pose.position.y);
  newTarget.Z(goal->goalPose.pose.position.z + 1.05);
  this->target = newTarget;


  this->velocity = goal->linearspeed;

  if(goal->is_motion){
    is_motionfinished = false;
  }

  if (goal->linearspeed < 0.05){
    std::cout << " Linearspeed " << goal->linearspeed << " is to low setting to stand " << std::endl;
    motion_type = static_cast<std::string> ("stand");
    is_motionfinished = true;
  }


  if (goal->linearspeed > 0.5){
    std::cout << " Linearspeed " << goal->linearspeed << " is to high setting to run " << std::endl;
    motion_type = static_cast<std::string> ("run");
  }


  this->Reset();
  
  
  auto start = std::chrono::high_resolution_clock::now();
  while (!is_motionfinished);

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;

  result.executing_time = elapsed.count();
  aserver->setSucceeded(result);
}

/////////////////////////////////////////////////
void GazeboROSAnimation::Reset(){
  this->lastUpdate = 0;

  /*
  if (this->sdf && this->sdf->HasElement("target"))
    this->target = this->sdf->Get<ignition::math::Vector3d>("target");
  else
    this->target = ignition::math::Vector3d(0, -5, 1.2138);
    */

  auto skelAnims = this->actor->SkeletonAnimations();
  /*
  for (auto const& element : skelAnims) {
    std::cout << "animation Available " << element.first << "\n";
  }
  */

  if (skelAnims.find(motion_type) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << motion_type << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = motion_type;
    this->trajectoryInfo->duration = 1.0;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}

/////////////////////////////////////////////////
void GazeboROSAnimation::HandleObstacles(ignition::math::Vector3d &_pos)
{
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  {
    physics::ModelPtr model = this->world->ModelByIndex(i);
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
          model->GetName()) == this->ignoreModels.end())
    {
      ignition::math::Vector3d offset = model->WorldPose().Pos() -
        this->actor->WorldPose().Pos();
      double modelDist = offset.Length();
      if (modelDist < 4.0)
      {
        double invModelDist = this->obstacleWeight / modelDist;
        offset.Normalize();
        offset *= invModelDist;
        _pos -= offset;
      }
    }
  }
}

/////////////////////////////////////////////////
void GazeboROSAnimation::OnUpdate(const common::UpdateInfo &_info)
{
  if (is_motionfinished){
    return;
  }

    // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();
  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d pos = this->target - pose.Pos();
  ignition::math::Vector3d rpy = pose.Rot().Euler();

  double distance = pos.Length();

  if (distance < 0.2){
    is_motionfinished = true;
    motion_type = static_cast<std::string>("stand");
    Reset();
    return;
  }

  // Choose a new target position if the actor has reached its current
  // target.
  /*
  if (distance < 0.3)
  {
    this->ChooseNewTarget();
    pos = this->target - pose.Pos();
  }
  */


  // Normalize the direction vector, and apply the target weight
  pos = pos.Normalize() * this->targetWeight;

  // Adjust the direction vector by avoiding obstacles
  this->HandleObstacles(pos);

  // Compute the yaw orientation
  ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();

  // Rotate in place, instead of jumping.
  if (std::abs(yaw.Radian()) > IGN_DTOR(10))
  {
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
        yaw.Radian()*0.001);
  }
  else
  {
    pose.Pos() += pos * this->velocity * dt;
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
  }

  // Make sure the actor stays within bounds
  /*
  pose.Pos().X(std::max(-3.0, std::min(3.5, pose.Pos().X())));
  pose.Pos().Y(std::max(-10.0, std::min(2.0, pose.Pos().Y())));
  pose.Pos().Z(1.2138);
  */

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (pose.Pos() -
      this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
    (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
}


void GazeboROSAnimation::QueueThread(){
    static const double timeout = 0.01;
    while (ros::ok()){
        //ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
        my_callback_queue.callAvailable(ros::WallDuration());
       //this->rosQueue.callAvailable(ros::WallDuration(timeout));
       ros::Duration(0.5).sleep();
      }
}