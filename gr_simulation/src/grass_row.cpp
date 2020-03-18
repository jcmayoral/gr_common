#include <grass_row.h>
using namespace gazebo;

GrassRow::GrassRow(){
    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
        ros::init_options::NoSigintHandler);
    }
}

void GrassRow::SetSizeZ(const double &_sz){
    //this->link->SetAngularVel(ignition::math::Vector3<double>(0.0,0.0,ang_velocity));
}

void GrassRow::SetSizeX(const double &_sx){
    //this->link->SetLinearVel(ignition::math::Vector3<double>(lin_velx,lin_vely,0.0));
}
void GrassRow::SetSizeY(const double &_sy){
    //this->link->SetLinearVel(ignition::math::Vector3<double>(lin_velx,lin_vely,0.0));
}

void GrassRow::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
    // Just output a message for now
    std::cerr << "\nThe custom plugin is attach to model[" <<
    _model->GetName() << "]\n";

    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(_model->GetWorld()->Name());
    visPub = this->node->Advertise<msgs::Visual>("~/visual", 10);

    // Set the visual's name. This should be unique.
    visualMsg.set_name("__RED_CYLINDER_VISUAL__");

    // Set the visual's parent. This visual will be attached to the parent
    visualMsg.set_parent_name(_model->GetScopedName());

    // Create a cylinder
    geomMsg = visualMsg.mutable_geometry();
    geomMsg->set_type(msgs::Geometry::CYLINDER);
    geomMsg->mutable_cylinder()->set_radius(1);
    geomMsg->mutable_cylinder()->set_length(.1);

    // Set the material to be bright red
    visualMsg.mutable_material()->mutable_script()->set_name(
        "Gazebo/RedGlow");

    // Set the pose of the visual relative to its parent
    msgs::Set(visualMsg.mutable_pose(),
        ignition::math::Pose3d(0, 0, 0.6, 0, 0, 0));

    // Don't cast shadows
    visualMsg.set_cast_shadows(false);

    visPub->Publish(visualMsg);

    /*
    if (_sdf->HasElement("ang_velocity")){
        ang_velocity = _sdf->Get<double>("ang_velocity");
    }

    if (_sdf->HasElement("lin_velx")){0
        lin_velx = _sdf->Get<double>("lin_velx");
    }

    if (_sdf->HasElement("lin_vely")){
        lin_vely = _sdf->Get<double>("lin_vely");
    }
    */


    //CallBack for Gazebo
    this->model = _model;
    this->link = _model->GetLinks()[0];
    std::string topicName = "~/" + this->model->GetName() + "/size";

    //Once all setup is finished
    //Callback for ROS
    //this->nh.reset(new ros::NodeHandle("gazebo_client"));
    this->nh = boost::make_shared<ros::NodeHandle>();

    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Vector3>(
            "/" + this->model->GetName() + "/vel_cmd",1,
            boost::bind(&GrassRow::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);

    this->rosPub = this->nh->advertise<std_msgs::Bool>( "/" + this->model->GetName() + "/rrrrr", 1);
    this->rosSub = this->nh->subscribe(so);

    //update pose TODO fancy stuff
    // Spin up the queue helper thread.
    this->rosQueueThread = std::thread(std::bind(&GrassRow::QueueThread, this));
}

void GrassRow::OnRosMsg(const geometry_msgs::Vector3ConstPtr &_msg){
    //TOBE REMOVED ONCE TIMER WORKS
    /*
    msgs::Visual visual_msg;
    msgs::Geometry geo_msg;
    visual_msg = this->link->GetVisualMessage("visual");
    msgs::Vector3d v3d;
    geo_msg = visual_msg.geometry(); 
    v3d = geo_msg.box().size();
    std::cout << v3d.x() << std::endl;
    std::cout << v3d.y() << std::endl;
    std::cout << v3d.z() << std::endl;
    */
        // Create a cylinder
    geomMsg->set_type(msgs::Geometry::BOX);
    //geomMsg->mutable_box()->set_size(1);
    msgs::Vector3d v3d = geomMsg->mutable_box()->size();
     std::cout << v3d.x() << std::endl;
    std::cout << v3d.y() << std::endl;
    std::cout << v3d.z() << std::endl;
    //geomMsg->mutable_box()->set_size(1);
    //geomMsg->mutable_cylinder()->set_length(.1);

    // Set the material to be bright red
    //visualMsg.mutable_material()->mutable_script()->set_name(
    // ..   "Gazebo/RedGlow");

    // Set the pose of the visual relative to its parent
    //msgs::Set(visualMsg.mutable_pose(),
    //   ignition::math::Pose3d(10, 0, 0.6, 0, 0, 0));

    msgs::Set(visualMsg.mutable_geometry()->mutable_size(),
        ignition::math::Vector3d(10, 10, 0.6));

    // Don't cast shadows
    //visualMsg.set_cast_shadows(false);

    visPub->Publish(visualMsg);

    //this->SetSizeX(_msg->x);
    //this->SetSizeY(_msg->y);
    //this->SetSizeZ(_msg->z);
}

void GrassRow::QueueThread(){
    static const double timeout = 0.01;
    while (this->nh->ok()){
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}