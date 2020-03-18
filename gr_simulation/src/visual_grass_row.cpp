#include <visual_grass_row.h>
#include <gtest/gtest.h>


using namespace gazebo;

VisualGrassRow::VisualGrassRow(){
    // Initialize ros, if it has not already bee initialized.
}

VisualGrassRow::~VisualGrassRow(){
    this->nh.reset();
    this->model.reset();
}

void VisualGrassRow::SetSizeZ(const double &_sz){
    //this->link->SetAngularVel(ignition::math::Vector3<double>(0.0,0.0,ang_velocity));
}

void VisualGrassRow::SetSizeX(const double &_sx){
    //this->link->SetLinearVel(ignition::math::Vector3<double>(lin_velx,lin_vely,0.0));
}
void VisualGrassRow::SetSizeY(const double &_sy){
    //this->link->SetLinearVel(ignition::math::Vector3<double>(lin_velx,lin_vely,0.0));
}

void VisualGrassRow::Init(){
    std::cout << "init" << std::endl;
    OnMsg();
}

void VisualGrassRow::OnUpdate(){
        std::cout << "onupdate" << std::endl;
}

void VisualGrassRow::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf){

//Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
    if (!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
        ros::init_options::NoSigintHandler);
    }

    //CallBack for Gazebo
    this->model = _parent;  
    model_id = this->model->Name();  
    std::cout << model_id << std::endl;

    // Just output a message for now
    //std::cerr << "\nThe custom plugin is attach to model[" <<this->model->Name() << "]\n";


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

    //Crashes visual plugin threating?
    /*
    std::string topicName = "grassrowsize";
    std::cout << "TOPIC NAME " << topicName << std::endl;
    //Once all setup is finished
    //Callback for ROS
    this->nh.reset(new ros::NodeHandle("gazebo_client"));
    this->nh = boost::make_shared<ros::NodeHandle>();

    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Bool>(
            topicName,1,
            boost::bind(&VisualGrassRow::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);

    //this->rosPub = this->nh->advertise<std_msgs::Bool>( "/" + model_id + "/notification", 1);
    this->rosSub = this->nh->subscribe(so);
    ROS_INFO("EVERYTHING FINE");
    //update pose TODO fancy stuff
    // Spin up the queue helper thread.
    this->rosQueueThread = std::thread(std::bind(&VisualGrassRow::QueueThread, this));
    */
}

void VisualGrassRow::OnMsg(){
    //TOBE REMOVED ONCE TIMER WORKS
    msgs::Visual visual_msg;
    msgs::Geometry geo_msg;
    msgs::Vector3d v3d;
    ignition::math::Vector3d new_scale;
    new_scale = this->model->Scale();
    std::cout << "SX" << new_scale.X() << std::endl;
    std::cout << "SY" << new_scale.Y() << std::endl;
    std::cout << "SZ " << new_scale.Z() << std::endl;
    new_scale.Z() = 100.0;
    new_scale.Y() = 100.0;
    new_scale.X() = 100.0;
    this->model->SetScale(new_scale);
    sleep(5.0);
    new_scale = this->model->Scale();
    std::cout << "NSX" << new_scale.X() << std::endl;
    std::cout << "NSY" << new_scale.Y() << std::endl;
    std::cout << "NSZ " << new_scale.Z() << std::endl;
    ignition::math::Pose3d update_pose = ignition::math::Pose3d(1,1,-1,0,0,0);
    //this->model->SetWorldPose(update_pose);
    sleep(5.0);
    //this->model->SetVisible(false);
    //this->link->UpdateVisualMsg();
    //geo_msg = visual_msg.geometry(); 
    //v3d = geo_msg.box().size();

    //this->model->Update();
    //this->SetSizeX(_msg->x);
    //this->SetSizeY(_msg->y);
    //this->SetSizeZ(_msg->z);
}

void VisualGrassRow::QueueThread(){
    static const double timeout = 0.01;
    while (this->nh->ok()){
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}