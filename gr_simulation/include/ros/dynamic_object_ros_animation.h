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

#ifndef GAZEBO_PLUGINS_GAZEBOROSANIMATION_H_
#define GAZEBO_PLUGINS_GAZEBOROSANIMATION_H_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"

#include <gr_action_msgs/SimMotionPlannerAction.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>


namespace gazebo
{
  class GZ_PLUGIN_VISIBLE GazeboROSAnimation : public ModelPlugin
  {
    const std::list<std::string> AVAILABLEMOTIONS = std::list<std::string>({"moonwalk", "run", "sit_down",
                                          "sitting", "stand_up", "stand", "talk_a", "talk_b", "walk"});

    /// \brief Constructor
    public: GazeboROSAnimation();
    virtual ~GazeboROSAnimation(){
      this->rosPub.shutdown();

      for (auto c: this->connections){
        c.reset();
      }
    }

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Reset();

    /// \brief Function that is called every update cycle.
    /// \param[in] _info Timing information
    private: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Helper function to avoid obstacles. This implements a very
    /// simple vector-field algorithm.
    /// \param[in] _pos Direction vector that should be adjusted according
    /// to nearby obstacles.
    private: void HandleObstacles(ignition::math::Vector3d &_pos);

    /// \brief Pointer to the parent actor.
    private: physics::ActorPtr actor;

    /// \brief Pointer to the world, for convenience.
    private: physics::WorldPtr world;

    /// \brief Pointer to the sdf element.
    private: sdf::ElementPtr sdf;

    /// \brief Velocity of the actor
    private: ignition::math::Vector3d velocity;

    /// \brief List of connections
    private: std::vector<event::ConnectionPtr> connections;

    /// \brief Current target location
    private: ignition::math::Vector3d target;

    /// \brief Target location weight (used for vector field)
    private: double targetWeight = 1.0;

    /// \brief Obstacle weight (used for vector field)
    private: double obstacleWeight = 1.0;

    /// \brief Time scaling factor. Used to coordinate translational motion
    /// with the actor's walking animation.
    private: double animationFactor = 1.0;

    /// \brief Time of the last update.
    private: common::Time lastUpdate;

    /// \brief List of models to ignore. Used for vector field
    private: std::vector<std::string> ignoreModels;

    /// \brief Custom trajectory info.
    private: physics::TrajectoryInfoPtr trajectoryInfo;


    private: boost::shared_ptr<actionlib::SimpleActionServer<gr_action_msgs::SimMotionPlannerAction>> aserver;
    private: ros::CallbackQueue my_callback_queue;

    public:  void executeCB(const gr_action_msgs::SimMotionPlannerGoalConstPtr &goal);
    private: physics::ModelPtr model;
    private: void QueueThread();
    private: std::thread rosQueueThread;
    private: std::string motion_type;
    private: bool is_motionfinished;

    private: ros::Publisher rosPub;

    ignition::math::Pose3d startpose;

  };
}
#endif
