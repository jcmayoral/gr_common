#ifndef _GR_DYN_OBSTACLE_PLUGIN_HH_
#define _GR_DYN_OBSTACLE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>


#include <thread>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Twist.h>

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

    //For ROS
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
 
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
        this->SetLinearVelocityX(_msg->x());
        this->SetLinearVelocityY(_msg->y());
        this->SetAngVelocity(_msg->z());
    }

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
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
        

        //CallBack for Gazebo
        this->model = _model;
        this->model->SetGravityMode(false);
        this->link = _model->GetLinks()[0];
        this->link->SetLinearVel(ignition::math::Vector3<double>(lin_velx,lin_vely,0.0));
        this->link->SetAngularVel(ignition::math::Vector3<double>(0.0,0.0,ang_velocity));
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init(this->model->GetWorld()->Name());
        std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";
        this->sub = this->node->Subscribe(topicName,&GRDynObstaclePlugin::OnMsg, this);


        //Callback for ROS
        // Initialize ros, if it has not already bee initialized.
        if (!ros::isInitialized()){
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
        }
        // Create our ROS node. This acts in a similar manner to
        // the Gazebo node
        this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
        // Create a named topic, and subscribe to it.
        ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
                "/" + this->model->GetName() + "/vel_cmd",1,
                boost::bind(&GRDynObstaclePlugin::OnRosMsg, this, _1),
                ros::VoidPtr(), &this->rosQueue);
        this->rosSub = this->rosNode->subscribe(so);

        // Spin up the queue helper thread.
        this->rosQueueThread =
        std::thread(std::bind(&GRDynObstaclePlugin::QueueThread, this));
    }


    public: void OnRosMsg(const geometry_msgs::TwistConstPtr &_msg){
        this->SetLinearVelocityX(_msg->linear.x);
        this->SetLinearVelocityY(_msg->linear.y);
        this->SetAngVelocity(_msg->angular.z);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread(){
        static const double timeout = 0.01;
        while (this->rosNode->ok()){
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
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
