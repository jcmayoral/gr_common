#ifndef _GAZEBO_DYNAMIC_OBSTACLE_HH_
#define _GAZEBO_DYNAMIC_OBSTACLE_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>

namespace gazebo
{
  /// \brief A plugin to control a GR Dynamic Obstacle sensor.
  class GazeboDynamicObject : public ModelPlugin
  {
    /// \brief Constructor
    public: 
        GazeboDynamicObject();
        void SetAngVelocity(const double &_vel);
        void SetLinearVelocityX(const double &_vel);
        void SetLinearVelocityY(const double &_vel);

        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        void OnMsg(ConstVector3dPtr &_msg);
        void OnUpdate();

    private: 
        void QueueThread();
        /// \brief Pointer to the model.
        physics::ModelPtr model;
        physics::LinkPtr link;
        /// \brief A node used for transport
        transport::NodePtr node;
        /// \brief A subscriber to a named topic.
        transport::SubscriberPtr sub;
        transport::PublisherPtr pub;

        ignition::math::Pose3<double> current_pose;
        double ang_velocity = 0;
        double lin_velx = 0;
        double lin_vely = 0;
        event::ConnectionPtr updateConnection;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(GazeboDynamicObject)
}
#endif


