#ifndef _GAZEBO_VISUAL_GRASS_ROW_HH_
#define _GAZEBO_VISUAL_GRASS_ROW_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include "grasscutter.pb.h"

namespace gazebo
{
  typedef const boost::shared_ptr<const gr_simulation_msgs::msgs::GrassCutterRequest> GrassCutterRequestPtr;
  /// \brief A plugin to control a GR Dynamic Obstacle sensor.
  class VisualGrassRow : public VisualPlugin
  {
    /// \brief Constructor
    public: 
        VisualGrassRow();
        ~VisualGrassRow();

        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        //virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf);
        virtual void Init();
    private: 
        void OnEvent();
        void OnRequest(GrassCutterRequestPtr &event);
        /// \brief Pointer to the model.
        rendering::VisualPtr model;
        std::string model_id;
        //physics::LinkPtr link;
        
        ignition::math::Pose3<double> current_pose;
        double ang_velocity = 0;
        double lin_velx = 0;
        double lin_vely = 0;
        sdf::ElementPtr visualElem;

        int access_counter;

        bool is_cut;
        /// \brief A node used for transport
        transport::NodePtr node;
        transport::SubscriberPtr sub;

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_VISUAL_PLUGIN(VisualGrassRow)
}
#endif

