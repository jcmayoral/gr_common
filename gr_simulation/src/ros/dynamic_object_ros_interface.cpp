#include <ros/dynamic_object_ros_interface.h>
using namespace gazebo;

GazeboROSDynamicObject::GazeboROSDynamicObject(): is_ok(true), motionplanner{}{
    // Initialize ros, if it has not already bee initialized
    if (!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client");//,ros::init_options::NoSigintHandler);
        ros::spinOnce();
    }

}

void GazeboROSDynamicObject::executeCB(const gr_action_msgs::SimMotionPlannerGoalConstPtr &goal){
    std::cout << "OK execute cb " << this->model->GetName() << std::endl;
    gr_action_msgs::SimMotionPlannerFeedback feedback;
    gr_action_msgs::SimMotionPlannerResult result;

    /*if (this->model->GetName().compare(goal->object_id)){
        std::cout<<  "WRONG ID "<<std::endl;
        aserver->setAborted();
        return;
    }*/

    if (aserver->isPreemptRequested() || !ros::ok()){
        aserver->setPreempted();
        return;
    }

    //Set Start Pose
    ignition::math::Pose3d pose;
    if(goal->setstart){
        pose.Pos().X() = goal->startpose.pose.position.x;
        pose.Pos().Y() = goal->startpose.pose.position.y;
        pose.Pos().Z() = goal->startpose.pose.position.z;
        pose.Rot().Euler(0,0,tf2::getYaw(goal->startpose.pose.orientation));

        this->link->SetWorldPose(pose, true, true);
    }
    // publish the feedback
    if (!goal->is_motion){
      aserver->publishFeedback(feedback);
      aserver->setSucceeded(result);
      return;
    }

    msgs::Vector3d* newTarget = new msgs::Vector3d();

    newTarget->set_x(goal->goalPose.pose.position.x);
    newTarget->set_y(goal->goalPose.pose.position.y);
    newTarget->set_z(tf2::getYaw(goal->goalPose.pose.orientation));



    gazebo::transport::NodePtr node(new gazebo::transport::Node);
    node->Init();

    auto start = std::chrono::high_resolution_clock::now();
    bool res = this->motionplanner(node,this->model->GetName(), 100.0, newTarget);
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;

    //path = this->motionplanner.getSBPLPath();

    result.executing_time = elapsed.count();

    if (res){
        aserver->setSucceeded(result);
    }
    else{
        aserver->setAborted(result);
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

    if (_sdf->HasElement("lin_velx")){
        lin_velx = _sdf->Get<double>("lin_velx");
    }

    if (_sdf->HasElement("lin_vely")){
        lin_vely = _sdf->Get<double>("lin_vely");
    }

    if (_sdf->HasElement("primfilepath")){
        std::string filename = _sdf->Get<std::string>("primfilepath");
        std::cout << filename << std::endl;
        motionplanner.setPrimitivesFilename(filename);
    }

    if (_sdf->HasElement("resolution")){
        double resolution = _sdf->Get<double>("resolution");
        std::cout << "set resolution " << resolution << std::endl;
        motionplanner.setResolution(resolution);
    }

    if (_sdf->HasElement("mapsize")){
        auto mps = _sdf->Get<double>("mapsize");
        mapsize = mps;
    }

    //CallBack for Gazebo
    this->model = _model;
    this->model->SetGravityMode(false);
    this->link = _model->GetLinks()[0];
    this->link->SetLinearVel(ignition::math::Vector3<double>(lin_velx,lin_vely,0.0));
    this->link->SetAngularVel(ignition::math::Vector3<double>(0.0,0.0,ang_velocity));

    //CallBack for Gazebo used by Motion Planner
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->Name());

    std::string pubtopicName = "/" + this->model->GetName() + "/odom";
    this -> pub = this->node->Advertise<msgs::Pose>(pubtopicName);
    std::string subtopicName = "/" + this->model->GetName() + "/vel_cmd";
    this->sub = this->node->Subscribe(subtopicName,&GazeboROSDynamicObject::OnMsg, this);
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboROSDynamicObject::OnUpdate, this));


    //Callback for ROS
    ros::NodeHandle nh;// = boost::make_shared<ros::NodeHandle>("~");
    nh.setCallbackQueue(&my_callback_queue);
    aserver = boost::make_shared<actionlib::SimpleActionServer<gr_action_msgs::SimMotionPlannerAction>>(nh, std::string("SimMotionPlanner")+"/" + this->model->GetName(),
                                                                boost::bind(&GazeboROSDynamicObject::executeCB, this, _1), false);

    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
            "/" + this->model->GetName() + "/vel_cmd",1,
            boost::bind(&GazeboROSDynamicObject::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
    // Create a named topic, and subscribe to it.
    /*
    ros::SubscribeOptions so = ros::SubscribeOptions::create<visualization_msgs::Marker>(
            "/" + this->model->GetName() + "/position",1,
            boost::bind(&GazeboROSDynamicObject::OnRosMsg2, this, _1),
            ros::VoidPtr(), &this->rosQueue);
    */
    //this->rosPub = nh.advertise<std_msgs::Bool>( "/" + this->model->GetName() + "/rrrrr", 1);
    this->rosSub = nh.subscribe(so);
    std::cout << "MODEL NAME " << this->model->GetName() << std::endl;

    // Spin up the queue helper thread.
    this->rosQueueThread = std::thread(std::bind(&GazeboROSDynamicObject::QueueThread, this));
    aserver->start();
    //ros::spinOnce();

}

void GazeboROSDynamicObject::OnMsg(ConstVector3dPtr &_msg){
    this->SetLinearVelocityX(_msg->x());
    this->SetLinearVelocityY(_msg->y());
    this->SetAngVelocity(_msg->z());
    //current_pose = this->model->WorldPose();
    //std::cout <<"X "<< current_pose.Pos().X();
}

void GazeboROSDynamicObject::OnUpdate(){
    current_pose = this->link->WorldPose();//this->model->WorldPose();
    this->pub->Publish(gazebo::msgs::Convert(current_pose));
}


void GazeboROSDynamicObject::goalCB(){
    gr_action_msgs::SimMotionPlannerGoal goal = *(aserver->acceptNewGoal());
    gr_action_msgs::SimMotionPlannerFeedback feedback;
    gr_action_msgs::SimMotionPlannerResult result;

    if (aserver->isPreemptRequested() || !ros::ok()){
        aserver->setPreempted();
    }

    aserver->publishFeedback(feedback);

    //gazebo::transport::NodePtr node(new gazebo::transport::Node);
    //node->Init();

    bool res = this->motionplanner(node,this->model->GetName(), mapsize);
    if (res){
        aserver->setSucceeded(result);
    }
    else{
        aserver->setAborted(result);
    }

}

void GazeboROSDynamicObject::OnRosMsg(const geometry_msgs::TwistConstPtr &_msg){
    //TOBE REMOVED ONCE TIMER WORKS
    this->updatePose(ros::TimerEvent());

    this->SetLinearVelocityX(_msg->linear.x);
    this->SetLinearVelocityY(_msg->linear.y);
    this->SetAngVelocity(_msg->angular.z);
}

void GazeboROSDynamicObject::OnRosMsg2(const visualization_msgs::MarkerConstPtr _msg){
    //TOBE REMOVED ONCE TIMER WORKS
    this->updatePose(ros::TimerEvent());
    ////this->SetLinearVelocityX(_msg->linear.x);
    //this->SetLinearVelocityY(_msg->linear.y);
    //this->SetAngVelocity(_msg->angular.z);
    ignition::math::Pose3d pose;
    pose.Pos().X(_msg->pose.position.x);
    pose.Pos().Y(_msg->pose.position.y);
    //this->model->SetWorldPose(pose, true, true);
    this->link->SetWorldPose(pose, true, true);

}

 void GazeboROSDynamicObject::updatePose(const ros::TimerEvent& event){
    current_pose = this->model->WorldPose();
    //std_msgs::Bool msg;
    //this->rosPub.publish(msg);
}

void GazeboROSDynamicObject::QueueThread(){
    static const double timeout = 0.01;
    while (is_ok){
        //ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
        my_callback_queue.callAvailable(ros::WallDuration());
       this->rosQueue.callAvailable(ros::WallDuration(timeout));
       ros::Duration(0.5).sleep();
      }
}
