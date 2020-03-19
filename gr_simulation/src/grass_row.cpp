#include <grass_row.h>
using namespace gazebo;

GrassRow::GrassRow(): gznode(new transport::Node()){
    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
        ros::init_options::NoSigintHandler);
    }
}

GrassRow::~GrassRow(){
    //
    this->nh->shutdown();
    this->rosSub.shutdown();
    this->rosQueueThread.detach();
    std::cout << "e1";
    this->nh.reset();
    std::cout << "e2";
    //delete this->nh;
    this->model.reset();
    this->link.reset();
    std::cout << "e3";
}

void GrassRow::Init(){
}

void GrassRow::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
    // Just output a message for now
    std::cerr << "\nThe custom plugin is attach to model[" <<
    _model->GetName() << "]\n";

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

    //Copying Poiners
    this->model = _model;
    this->link = _model->GetLinks()[0];
    std::string topicName = "/" + this->model->GetName() + "/event";


    //this->gznode = transport::NodePtr(new transport::Node());
    this->gznode->Init("grasscutter");
    this->gzsub = this->gznode->Subscribe("~/test",&GrassRow::OnRequest, this);
    gazebo::common::Time::MSleep(10);

    //Once all setup is finished
    //Callback for ROS
    //this->nh.reset(new ros::NodeHandle("gazebo_client"));
    //this->nh.reset(new ros::NodeHandle(this->model->GetName()));
    this->nh = boost::make_shared<ros::NodeHandle>();
    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Bool>(
            topicName,1,
           boost::bind(&GrassRow::OnRosMsg, this, _1),
         ros::VoidPtr(), &this->rosQueue);
    //this->rosPub = this->nh->advertise<std_msgs::Empty>( "/" + this->model->GetName() + "/rrrrr", 1);
    this->rosSub = this->nh->subscribe(so);
    //this->rosSub = this->nh.subscribe(topicName, 1, &GrassRow::OnRosMsg, this);
    // Spin up the queue helper thread.
    this->rosQueueThread = std::thread(std::bind(&GrassRow::QueueThread, this));
}

void GrassRow::OnRequest(GrassCutterRequestPtr &event){
    std::cout << "onRequest " << std::endl;
    OnEvent(true);
}


void GrassRow::OnRosMsg(const std_msgs::BoolConstPtr& msg){
    //TOBE REMOVED ONCE TIMER WORKS
    std::cout << "CB " << std::endl;
    OnEvent(msg->data);
    //this->model->Update();
}

void GrassRow::OnEvent(bool state){
    ignition::math::Vector3d nscale;
    nscale.X() = 1.0;
    nscale.Y() = 1.0;
    nscale.Z() = 1.0;
    double wz = 0.0;
    if (state){
        nscale.Z() = 0.5;
        wz = -3.0;
    }
   //this->model->SetScale(nscale,true);

   uint32_t visualid;
   if (this->link->VisualId("visual", visualid)){
       ignition::math::Pose3d pose;//(0,0,wz,0,0,0);
       std::cout << "a";
       if(this->link->VisualPose(visualid,pose)){
           std::cout << pose.Pos().Z() << std::endl;
           pose.Pos().Z() = wz;
           if(this->link->SetVisualPose(visualid,pose)){
               std::cout << "w1";
            }
       }

   }

   //this->model->SetEnabled(false);
   //this->model->Update();
   //;
   //this->model->SetLinkWorldPose(pose, this->link);
}

void GrassRow::QueueThread(){
    static const double timeout = 0.01;
    while (this->nh->ok()){
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}