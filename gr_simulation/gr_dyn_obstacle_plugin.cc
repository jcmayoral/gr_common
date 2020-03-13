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
#include <std_msgs/Bool.h>

namespace gazebo
{
  /// \brief A plugin to control a GR Dynamic Obstacle sensor.
  class GRDynObstaclePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: GRDynObstaclePlugin() {
        // Initialize ros, if it has not already bee initialized.
        if (!ros::isInitialized()){
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
        }
    }

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




        //Once all setup is finished
        //Callback for ROS
        //this->nh.reset(new ros::NodeHandle("gazebo_client"));
        this->nh = boost::make_shared<ros::NodeHandle>();
     
        //NOT WORKING TEST WITH full simulation
        //this->poseTimer = this->nh->createTimer(ros::Duration(0.1), &GRDynObstaclePlugin::updatePose, this);
        ROS_INFO("HERE");
        // Create a named topic, and subscribe to it.
        ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
                "/" + this->model->GetName() + "/vel_cmd",1,
                boost::bind(&GRDynObstaclePlugin::OnRosMsg, this, _1),
                ros::VoidPtr(), &this->rosQueue);
     
        this->rosPub = this->nh->advertise<std_msgs::Bool>( "/" + this->model->GetName() + "/rrrrr", 1);
        this->rosSub = this->nh->subscribe(so);
     
        ROS_INFO("THERE");
        //update pose TODO fancy stuff
        // Spin up the queue helper thread.
        this->rosQueueThread =
        std::thread(std::bind(&GRDynObstaclePlugin::QueueThread, this));
    }


    public: void OnRosMsg(const geometry_msgs::TwistConstPtr &_msg){
        //TOBE REMOVED ONCE TIMER WORKS
        this->updatePose(ros::TimerEvent());

        this->SetLinearVelocityX(_msg->linear.x);
        this->SetLinearVelocityY(_msg->linear.y);
        this->SetAngVelocity(_msg->angular.z);
    }

    public: void updatePose(const ros::TimerEvent& event){
        ROS_INFO("GERER");
        current_pose = this->model->WorldPose();
        std::cout << "A" << std::endl;
        std::cout <<"X "<< current_pose.Pos().X();
        std::cout << "B" << std::endl;
        std_msgs::Bool msg;
        this->rosPub.publish(msg);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread(){
        static const double timeout = 0.01;
        while (this->nh->ok()){
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
    }

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    private: physics::LinkPtr link;
    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    public: ignition::math::Pose3<double> current_pose;

    private: double ang_velocity = 0;
    private: double lin_velx = 0;
    private: double lin_vely = 0;

    //For ROS
    /// \brief A node use for ROS transpor
    private: ros::NodeHandlePtr nh;
    private: ros::Timer poseTimer;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;
    private: ros::Publisher rosPub;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
 


  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(GRDynObstaclePlugin)
}
#endif
