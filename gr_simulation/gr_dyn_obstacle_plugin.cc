#ifndef _GR_DYN_OBSTACLE_PLUGIN_HH_
#define _GR_DYN_OBSTACLE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  /// \brief A plugin to control a GR Dynamic Obstacle sensor.
  class GRDynObstaclePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: GRDynObstaclePlugin() {}

    private: double ang_velocity = 0;
    private: double lin_velx = 0;
    private: double lin_vely = 0;
 
    public: void SetAngVelocity(const double &_vel){
        ang_velocity = _vel;
	this->link->SetAngularVel(ignition::math::Vector3<double>(0.0,0.0,ang_velocity));		 
    }
    public: void SetLinearVelocityX(const double &_vel){
        lin_velx = _vel;
	this->link->SetLinearVel(ignition::math::Vector3<double>(lin_velx,lin_vely,0.0));
    }
    public: void SetLinearVelocityY(const double &_vel){
        lin_vely = _vel;
	this->link->SetLinearVel(ignition::math::Vector3<double>(lin_velx,lin_vely,0.0));
    }

    private: void OnMsg(ConstVector3dPtr &_msg){
printf("%s","working callback");
	this->SetLinearVelocityX(_msg->x());
	this->SetLinearVelocityY(_msg->y());
	this->SetAngVelocity(_msg->z());
    }

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
    


      // Just output a message for now
      std::cerr << "\nThe velodyne plugin is attach to model[" <<
        _model->GetName() << "]\n";


      if (_sdf->HasElement("ang_velocity")){
	      ang_velocity = _sdf->Get<double>("ang_velocity");
      }
      if (_sdf->HasElement("lin_velx")){
	      lin_velx = _sdf->Get<double>("lin_velx");
      }
      if (_sdf->HasElement("lin_vely")){
	      lin_vely = _sdf->Get<double>("lin_vely");
      }
       // Safety check
       //if (_model->GetJointCount() == 0){
	//std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
    	//return;
      	// }
 
       // Store the model pointer for convenience.
       this->model = _model;
        // Get the first joint. We are making an assumption about the model
	// having one joint that is the rotational joint.
	//this->joint = _model->GetJoints()[0];
	//printf("%b" % this->model->GetGravityMode());
	this->model->SetGravityMode(false);
	//printf ("%d" % len(this->model->GetLinks()));
	this->link = _model->GetLinks()[0];

	// Setup a P-controller, with a gain of 0.1.
	//this->pid = common::PID(0.1, 0, 0);

	// Apply the P-controller to the joint.
	this->link->SetLinearVel(ignition::math::Vector3<double>(lin_velx,lin_vely,0.0));
	this->link->SetAngularVel(ignition::math::Vector3<double>(0.0,0.0,ang_velocity));
	//this->model->GetJointController()->SetVelocityPID(this->joint->GetScopedName(), this->pid);

	// Set the joint's target velocity. This target velocity is just
	// // for demonstration purposes.
	//this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), 10.0);
    // Create the node
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->Name());
    // Create a topic name
    std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";
    // Subscribe to the topic, and register a callback
    this->sub = this->node->Subscribe(topicName,&GRDynObstaclePlugin::OnMsg, this);
    }
    /// \brief Pointer to the model.
private: physics::ModelPtr model;

/// \brief Pointer to the joint.
private: physics::JointPtr joint;

private: physics::LinkPtr link;

/// \brief A PID controller for the joint.
private: common::PID pid;


	 /// \brief A node used for transport
private: transport::NodePtr node;

/// \brief A subscriber to a named topic.
private: transport::SubscriberPtr sub;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(GRDynObstaclePlugin)
}
#endif
