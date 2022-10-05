#include <ros/simple_ros_interface.h>
using namespace gazebo;

SimpleROSInterface::SimpleROSInterface(): is_ok{true}, flag{false}, tfBuffer(ros::Duration(5)),
                                tf2_listener(tfBuffer), dist2collision{1.5}, original_distance{10.0},
                                count{0}, is_collided{false}{
    // Initialize ros, if it has not already bee initialized
    std::cout << "CONSTRUCTOR 10 "<<std::endl;
    desiredspeed = new ignition::math::Vector3d();
    if (true){//!ros::isInitialized()){
        std::cout << "CONSTRUCTOR 10 initializing"<<std::endl;

        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "simple_human");//,ros::init_options::NoSigintHandler);
        //ros::spinOnce();
    }
        std::cout << "CONSTRUCTOR 11 "<<std::endl;


}


void SimpleROSInterface::executeCB(const gr_action_msgs::SimMotionPlannerGoalConstPtr &goal){
    count = 0;
    is_collided = false;
    std::cout << "OK execute cb " << this->model->GetName() << std::endl;
    original_distance = goal->original_distance;
    forward = true;
    gr_action_msgs::SimMotionPlannerFeedback feedback;
    gr_action_msgs::SimMotionPlannerResult result;
    ignition::math::Vector3 curAngularVel = this->link->WorldAngularVel();

    this->link->SetAngularVel(ignition::math::Vector3<double>(0.0,0.0,0.0));
    this->link->SetLinearVel(ignition::math::Vector3<double>(0.0,0.0,0.0));
    this->model->ResetPhysicsStates();

    //std::cout << curAngularVel << "ANGULAR VEL" << std::endl;

    /*if (this->model->GetName().compare(goal->object_id)){
        std::cout<<  "WRONG ID "<<std::endl;
        aserver->setAborted();
        return;
    }*/

    if (aserver->isPreemptRequested() || !ros::ok()){
        aserver->setPreempted();
        return;
    }
    //aserver->acceptNewGoal();
    //Set Start Pose
    //ignition::math::Pose3d startpose;

    if(goal->setstart){
        startpose.Pos().X() = goal->startpose.pose.position.x;
        startpose.Pos().Y() = goal->startpose.pose.position.y;
        startpose.Pos().Z() = goal->startpose.pose.position.z;
        startpose.Rot().Euler(0,0,tf2::getYaw(goal->startpose.pose.orientation));

        this->link->SetWorldPose(startpose, true, true);
    }

    dist2collision = goal->dist2collision;
    desiredspeed->Set(goal->linearspeed,goal->linearspeedy,0);

    this->SetLinearVelocityX(goal->linearspeed);
    this->SetLinearVelocityY(goal->linearspeedy);

    // publish the feedback
    /*
    if (goal->is_motion){
        aserver->publishFeedback(feedback);
        aserver->setSucceeded(result);
        return;
    }
    */
    ignition::math::Vector3<double> newTarget;
    newTarget.Set(goal->goalPose.pose.position.x,goal->goalPose.pose.position.y,0);
    ignition::math::Vector3<double> neworientation;
    neworientation.Set(0,0,tf2::getYaw(goal->goalPose.pose.orientation));

    endpose.Set(newTarget, neworientation);

    //msgs::Vector3d* newTarget = new msgs::Vector3d();
    //newTarget->set_x(goal->goalPose.pose.position.x);
    //newTarget->set_y(goal->goalPose.pose.position.y);
    //newTarget->set_z(tf2::getYaw(goal->goalPose.pose.orientation));

    //gazebo::transport::NodePtr node(new gazebo::transport::Node);
    //node->Init();

    auto start = std::chrono::high_resolution_clock::now();
    //bool res = this->motionplanner(node,this->model->GetName(), 100.0, newTarget);
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;

    //path = this->motionplanner.getSBPLPath();

    result.executing_time = elapsed.count();

    //if (res){
    aserver->setSucceeded(result);
}


void SimpleROSInterface::SetAngVelocity(const double &_vel){
    ang_velocity = _vel;
    this->link->SetAngularVel(ignition::math::Vector3<double>(0.0,0.0,ang_velocity));
}

void SimpleROSInterface::SetLinearVelocityX(const double &_vel){
    lin_velx = _vel;
    this->link->SetLinearVel(ignition::math::Vector3<double>(lin_velx,lin_vely,0.0));
}

void SimpleROSInterface::SetLinearVelocityY(const double &_vel){
    lin_vely = _vel;
    this->link->SetLinearVel(ignition::math::Vector3<double>(lin_velx,lin_vely,0.0));
}

void SimpleROSInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
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

    //CallBack for Gazebo
    this->model = _model;
    this->model->SetGravityMode(false);
    this->link = _model->GetLinks()[0];
    this->link->SetLinearVel(ignition::math::Vector3<double>(lin_velx,lin_vely,0.0));
    this->link->SetAngularVel(ignition::math::Vector3<double>(0.0,0.0,ang_velocity));
    //CallBack for Gazebo used by Motion Planner
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->Name());

    //std::string pubtopicName = "/" + this->model->GetName() + "/odom";
    //this -> pub = this->node->Advertise<msgs::Pose>(pubtopicName);
    //std::string subtopicName = "/" + this->model->GetName() + "/vel_cmd";
    //this->sub = this->node->Subscribe(subtopicName,&SimpleROSInterface::OnMsg, this);
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&SimpleROSInterface::OnUpdate, this));

     if (!ros::isInitialized()){
        std::cout << "CONSTRUCTOR 10 initializing"<<std::endl;

        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "simple_human");//,ros::init_options::NoSigintHandler);
        //ros::spinOnce();
    }

    //Callback for ROS
    ros::NodeHandle nh;// = boost::make_shared<ros::NodeHandle>("~");
    nh.setCallbackQueue(&my_callback_queue);
    aserver = boost::make_shared<actionlib::SimpleActionServer<gr_action_msgs::SimMotionPlannerAction>>(nh, "SimMotionPlanner/" + this->model->GetName(),
                                                                boost::bind(&SimpleROSInterface::executeCB, this, _1), false);

    std::string topic_name = this->model->GetName() + "/human_collision";

    rosPub = nh.advertise<safety_msgs::HumanSafety>(topic_name, 1);

    std::string topic_name2 = this->model->GetName() + "/human_trigger";
    rosFinishPub = nh.advertise<std_msgs::Empty>(topic_name2, 1);

    // Spin up the queue helper thread.
    futureObj = exitSignal.get_future();
    aserver->start();
    this->rosQueueThread = std::thread(std::bind(&SimpleROSInterface::QueueThread, this), std::move(futureObj));
    //ros::spinOnce();


}

void SimpleROSInterface::OnUpdate(){
    current_pose = this->model->WorldPose();
    auto collision = this->link->GetCollision("box");
    //this->link->OnCollision(collision);
    if (collision == NULL){
        std::cout << "ERROR " << std::endl;
    }
    auto collisionstate = collision->GetState();
    //std::cout << "HERE "<<std::endl;

    geometry_msgs::TransformStamped camera_link_to_odom;
    geometry_msgs::TransformStamped odom_to_base_link;

    geometry_msgs::PoseStamped p0;
    geometry_msgs::PoseStamped p1;
    //geometry_msgs::PoseStamped pend;

    //tfBuffer.canTransform("base_link", "odom", ros::Time(0), ros::Duration(1.0) );

    try{
        //orientation of current odometry to map
        p0.header.stamp = ros::Time::now();
        p0.header.frame_id = "odom";
        p0.pose.position.x = current_pose.Pos().X();
        p0.pose.position.y = current_pose.Pos().Y();
        p0.pose.orientation.w = 1.0;
        odom_to_base_link = tfBuffer.lookupTransform("base_link", "odom", ros::Time(0));
        camera_link_to_odom = tfBuffer.lookupTransform("odom", "camera_link", ros::Time(0));
        tf2::doTransform(p0, p1, odom_to_base_link);

        /*
        pend.header.stamp = ros::Time::now();
        pend.header.frame_id = "odom";
        pend.pose.position.x = endpose.Pos().X();
        pend.pose.position.y = endpose.Pos().Y();
        pend.pose.orientation.w = 1.0;
        tf2::doTransform(pend, pend, odom_to_base_link);
        */
    }
    catch(tf2::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        return;
    }
    //double dist2robot = sqrt(pow(p1.pose.position.x,2)+pow(p1.pose.position.y,2));
    //In odom coordinates
    double dist2camera = sqrt(pow(p1.pose.position.x- camera_link_to_odom.transform.translation.x,2)+
                        pow(p1.pose.position.y- camera_link_to_odom.transform.translation.y,2));

    double dist2goal = sqrt(pow(current_pose.Pos().X() - endpose.Pos().X(),2) + pow(current_pose.Pos().Y() - endpose.Pos().Y(),2));
    std::cout << "Distance2camera " << dist2camera << std::endl;
    //std::cout << "Not Collide" << collisionstate.IsZero() << std::endl;

    /*
    tf2::Quaternion quat_tf;
    tf2::convert(pend.pose.orientation , quat_tf);
    tf2::Matrix3x3 m(quat_tf);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    std::cout << " DIFF " << yaw << std::endl;
    auto vx = this->link->RelativeLinearVel().X() * cos(yaw);
    auto vy = this->link->RelativeLinearVel().Y() * sin(yaw);
    this->link->SetLinearVel(ignition::math::Vector3<double>(vx,vy,0.0));
    */

    if (dist2goal <0.5 || dist2camera < dist2collision || dist2goal > original_distance){
        ROS_ERROR_STREAM("DONE Distance 2 robot"<< dist2camera );
        //this->link->SetAngularVel(ignition::math::Vector3<double>(0.0,0.0,0.0));
        //this->link->SetLinearVel(ignition::math::Vector3<double>(0.0,0.0,0.0));     
        //this->SetLinearVelocityX(0);
        //this->SetLinearVelocityY(0);   
        //this->model->ResetPhysicsStates();
        //this->model->Reset();

        //std::cout << startpose.Pos().X() << "::::" << endpose.Pos().X() << std::endl;
        if (dist2camera < dist2collision && !is_collided){
            is_collided = true;
            std::cout << "add offset to avoid collision"<< std::endl;
            startpose.Pos().Y() += 2.0;
            endpose.Pos().Y() += 2.0;
            startpose.Pos().X() += 2.0;
            endpose.Pos().X() += 2.0;
            safety_msgs::HumanSafety fb;
            fb.odom_pose.header = p0.header;
            fb.odom_pose.point = p0.pose.position;

            fb.base_link_pose.header = p1.header;
            fb.base_link_pose.point = p1.pose.position; 
            fb.inCollision = true;
            fb.distance = dist2camera;
            ROS_INFO_STREAM (fb);
            this->rosPub.publish(fb);
            this->rosFinishPub.publish(std_msgs::Empty());
        }
        
        //std::swap(endpose,startpose);
        //std::cout << startpose.Pos().X() << "::::" << endpose.Pos().X() << std::endl;

        //startpose.Rot().Z() += M_PI;// - startpose.Rot().Z();//
        // - startpose.Rot().Z();
        //if (startpose.Rot().Z() >= 2*M_PI){
        //    startpose.Rot().Z() -= (2*M_PI);//
        //}


        ignition::math::Vector3<double> orientation;
        orientation = startpose.Rot().Euler();
        orientation.Z() +=M_PI;
        if (orientation.Z()>=2*M_PI){
            orientation.Z() -= (2*M_PI);
        }
        startpose.Rot().Euler(orientation.X(), orientation.Y(), orientation.Z());

        this->link->SetWorldPose(startpose, true, true);

        endpose.Rot() = startpose.Rot();
        desiredspeed->X(-desiredspeed->X());
        desiredspeed->Y(-desiredspeed->Y());

        //endpose.Set(endpose.Pos(), endpose.Rot());

        this->SetLinearVelocityX(desiredspeed->X());
        this->SetLinearVelocityY(desiredspeed->Y());

        if(count==2){
            this->rosFinishPub.publish(std_msgs::Empty());
        }
        count++;
     }
    //this->pub->Publish(gazebo::msgs::Convert(current_pose));
}


 void SimpleROSInterface::updatePose(const ros::TimerEvent& event){
    current_pose = this->model->WorldPose();
    //std_msgs::Bool msg;
    //this->rosPub.publish(msg);
}

void SimpleROSInterface::QueueThread(){
    static const double timeout = 0.1;
    while (is_ok){
      ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
      my_callback_queue.callAvailable(ros::WallDuration());
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
      //ros::Duration(0.5).sleep();
      std::this_thread::sleep_for (std::chrono::milliseconds(500));
    }
    std::cout << "Dying" <<std::endl;
}
