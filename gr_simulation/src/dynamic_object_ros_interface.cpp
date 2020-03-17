#include <dynamic_object_ros_interface.h>
using namespace gazebo;

GazeboROSDynamicObject::GazeboROSDynamicObject(){
    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
        ros::init_options::NoSigintHandler);
    }
}

void GazeboROSDynamicObject::SetAngVelocity(const double &_vel){
    ang_velocity = _vel;
    this->link->SetAngularVel(ignition::math::Vector3<double>(0.0,0.0,ang_velocity));
}

void GazeboROSDynamicObject::SetLinearVelocityX(const double &_vel){
    lin_velx = _vel;
    this->link->SetLinearVel(ignition::math::Vector3<double>(lin_velx,lin_vely,0.0));
}
void GazeboROSDynamicObject::SetLinearVelocityY(const double &_vel){
    lin_vely = _vel;
    this->link->SetLinearVel(ignition::math::Vector3<double>(lin_velx,lin_vely,0.0));
}

void GazeboROSDynamicObject::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
    // Just output a message for now
    std::cerr << "\nThe custom plugin is attach to model[" <<
    _model->GetName() << "]\n";

    if (_sdf->HasElement("ang_velocity")){
        ang_velocity = _sdf->Get<double>("ang_velocity");
    }

    if (_sdf->HasElement("lin_velx")){0
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
    std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

    //Once all setup is finished
    //Callback for ROS
    //this->nh.reset(new ros::NodeHandle("gazebo_client"));
    this->nh = boost::make_shared<ros::NodeHandle>();

    //NOT WORKING TEST WITH full simulation
    //this->poseTimer = this->nh->createTimer(ros::Duration(0.1), &GRDynObstaclePlugin::updatePose, this);
    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
            "/" + this->model->GetName() + "/vel_cmd",1,
            boost::bind(&GazeboROSDynamicObject::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);

    this->rosPub = this->nh->advertise<std_msgs::Bool>( "/" + this->model->GetName() + "/rrrrr", 1);
    this->rosSub = this->nh->subscribe(so);

    //update pose TODO fancy stuff
    // Spin up the queue helper thread.
    this->rosQueueThread =
    std::thread(std::bind(&GazeboROSDynamicObject::QueueThread, this));
}

void GazeboROSDynamicObject::OnRosMsg(const geometry_msgs::TwistConstPtr &_msg){
    //TOBE REMOVED ONCE TIMER WORKS
    this->updatePose(ros::TimerEvent());

    this->SetLinearVelocityX(_msg->linear.x);
    this->SetLinearVelocityY(_msg->linear.y);
    this->SetAngVelocity(_msg->angular.z);
}

 void GazeboROSDynamicObject::updatePose(const ros::TimerEvent& event){
    current_pose = this->model->WorldPose();
    std::cout <<"X "<< current_pose.Pos().X();
    std_msgs::Bool msg;
    this->rosPub.publish(msg);
}

void GazeboROSDynamicObject::QueueThread(){
    static const double timeout = 0.01;
    while (this->nh->ok()){
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}