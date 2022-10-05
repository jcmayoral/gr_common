#ifndef _SIMPLE_ROS_INTERFACE_HH_
#define _SIMPLE_ROS_INTERFACE_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

//#include <gr_simulation/PersonMotionConfig.h>
#include <thread>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
//#include <dynamic_reconfigure/server.h>
#include <actionlib/server/simple_action_server.h>
#include <gr_action_msgs/SimMotionPlannerAction.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include <nav_msgs/Path.h>


#include <safety_msgs/HumanSafety.h>


#include <future>

namespace gazebo
{
  /// \brief A plugin to control a GR Dynamic Obstacle sensor.
  class SimpleROSInterface : public ModelPlugin
  {
    /// \brief Constructor
    public:
        SimpleROSInterface();
        virtual ~SimpleROSInterface(){
          is_ok = false;
          exitSignal.set_value();
          rosQueueThread.join();

          rosQueue.clear();
          rosQueue.disable();
          my_callback_queue.clear();
          my_callback_queue.disable();
          //aserver->shutdown();
          //delete aserver;
          rosSub.shutdown();
          rosPub.shutdown();
          poseTimer.stop();
          updateConnection.reset();
          std::cout << "destroyeda"<< std::endl;


          //sub->Unsubscribe();
          std::cout << "destroyed"<< std::endl;
        }

        void SetAngVelocity(const double &_vel);
        void SetLinearVelocityX(const double &_vel);
        void SetLinearVelocityY(const double &_vel);

        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        void updatePose(const ros::TimerEvent& event);
        //void dyn_reconfigureCB(gr_simulation::PersonMotionConfig &config, uint32_t level);

        void executeCB(const gr_action_msgs::SimMotionPlannerGoalConstPtr &goal);

        //void OnMsg(ConstVector3dPtr &_msg);
        void OnUpdate();

    private:
        //void OnRosMsg(const geometry_msgs::TwistConstPtr &_msg);
        //void OnRosMsg2(const visualization_msgs::MarkerConstPtr _msg);
        void QueueThread();
        /// \brief Pointer to the model.
        physics::ModelPtr model;
        physics::LinkPtr link;

        ignition::math::Pose3<double> current_pose;

        double ang_velocity = 0;
        double lin_velx = 0;
        double lin_vely = 0;
        double mapsize = 30.0;
        double dist2collision = 1.5;

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

        std::atomic_bool is_ok;

        //dynamic_reconfigure::Server<gr_simulation::PersonMotionConfig> dyn_server_;
        //dynamic_reconfigure::Server<gr_simulation::PersonMotionConfig>::CallbackType dyn_server_cb_;

        transport::NodePtr node;
        //transport::SubscriberPtr sub;
        transport::PublisherPtr pub;
        event::ConnectionPtr updateConnection;

        std::promise<void> exitSignal;
        std::future<void> futureObj;
        boost::shared_ptr<actionlib::SimpleActionServer<gr_action_msgs::SimMotionPlannerAction>> aserver;

        ignition::math::Pose3d startpose;
        ignition::math::Pose3d endpose;

        ignition::math::Vector3d* desiredspeed;

        bool forward;
        bool flag;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tf2_listener;

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(SimpleROSInterface)
}
#endif
