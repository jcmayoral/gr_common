#ifndef _GAZEBO_ROS_DYNAMIC_OBSTACLE_HH_
#define _GAZEBO_ROS_DYNAMIC_OBSTACLE_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <gr_action_msgs/SimMotionPlannerAction.h>
#include <actionlib/server/simple_action_server.h>

#include <thread>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <gazebo/motion_planner.h>

//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include <nav_msgs/Path.h>

namespace gazebo
{
  /// \brief A plugin to control a GR Dynamic Obstacle sensor.
  class GazeboROSDynamicObject : public ModelPlugin
  {
    /// \brief Constructor
    public: 
        GazeboROSDynamicObject();
        virtual ~GazeboROSDynamicObject(){
          aserver->shutdown();
          //delete aserver;
          aserver.reset();
          rosSub.shutdown();
          rosPub.shutdown();
          std::cout << "destroyed0"<< std::endl;
          is_ok = false;
          //rosQueueThread.join();
          std::cout << "destroyed"<< std::endl;
        }

        void SetAngVelocity(const double &_vel);
        void SetLinearVelocityX(const double &_vel);
        void SetLinearVelocityY(const double &_vel);

        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        void updatePose(const ros::TimerEvent& event);

        void executeCB(const gr_action_msgs::SimMotionPlannerGoalConstPtr &goal);

        void goalCB();

        void OnMsg(ConstVector3dPtr &_msg);
        void OnUpdate();
        void publishPath();


    private: 
        void OnRosMsg(const geometry_msgs::TwistConstPtr &_msg);
        void QueueThread();
        /// \brief Pointer to the model.
        physics::ModelPtr model;
        physics::LinkPtr link;
        
        ignition::math::Pose3<double> current_pose;
        double ang_velocity = 0;
        double lin_velx = 0;
        double lin_vely = 0;
        double mapsize = 30.0;

        //For ROS
        /// \brief A node use for ROS transpor
        ros::Timer poseTimer;
        /// \brief A ROS subscriber
        ros::Subscriber rosSub;
        ros::Publisher rosPub;
        /// \brief A ROS callbackqueue that helps process messages
        ros::CallbackQueue rosQueue;
        /// \brief A thread the keeps running the rosQueue
        std::thread rosQueueThread;
        ros::CallbackQueue my_callback_queue;
        
        bool is_ok;
        boost::shared_ptr<actionlib::SimpleActionServer<gr_action_msgs::SimMotionPlannerAction>> aserver;
        MotionPlanner motionplanner;
        //actionlib::SimpleActionServer<gr_action_msgs::SimMotionPlannerAction>* aserver;

        transport::NodePtr node;
        transport::SubscriberPtr sub;
        transport::PublisherPtr pub;
        event::ConnectionPtr updateConnection;

        std::vector<EnvNAVXYTHETALAT3Dpt_t> path;
        ros::Publisher path_pub;

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(GazeboROSDynamicObject)
}
#endif

