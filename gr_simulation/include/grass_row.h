#ifndef _GAZEBO_ROS_DYNAMIC_OBSTACLE_HH_
#define _GAZEBO_ROS_DYNAMIC_OBSTACLE_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>


#include <thread>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>

namespace gazebo
{
  /// \brief A plugin to control a GR Dynamic Obstacle sensor.
  class GrassRow : public ModelPlugin
  {
    /// \brief Constructor
    public: 
        GrassRow();
        void SetSizeZ(const double &_sz);
        void SetSizeX(const double &_sx);
        void SetSizeY(const double &_sy);

        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private: 
        void OnRosMsg(const geometry_msgs::Vector3ConstPtr &_msg);
        void QueueThread();
        /// \brief Pointer to the model.
        physics::ModelPtr model;
        physics::LinkPtr link;
        
        ignition::math::Pose3<double> current_pose;
        double ang_velocity = 0;
        double lin_velx = 0;
        double lin_vely = 0;

        //For ROS
        /// \brief A node use for ROS transpor
        ros::NodeHandlePtr nh;
        ros::Timer poseTimer;
        /// \brief A ROS subscriber
        ros::Subscriber rosSub;
        ros::Publisher rosPub;
        /// \brief A ROS callbackqueue that helps process messages
        ros::CallbackQueue rosQueue;
        /// \brief A thread the keeps running the rosQueue
        std::thread rosQueueThread;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(GrassRow)
}
#endif

