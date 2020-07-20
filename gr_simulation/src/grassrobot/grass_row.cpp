#include <gazebo/grass_row.h>
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

    bool use_ros=true;

    if (_sdf->HasElement("use_ros")){
        use_ros = _sdf->Get<bool>("use_ros");
    }

    this->model = _model;
    this->link = _model->GetLinks()[0];
    std::string topicName = "/" + this->model->GetName();
        
    //Copying Poiners
    if(!use_ros){
        //this->gznode = transport::NodePtr(new transport::Node());
        std::cout << "gazebo";
        this->gznode->Init();
        this->gzsub = this->gznode->Subscribe(topicName,&GrassRow::OnRequest, this);
    }

    if (use_ros){
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
}

void GrassRow::OnRequest(GrassCutterRequestPtr &event){
    std::cout << "onRequest " << std::endl;
    std::cout << event->cut() << std::endl;
    OnEvent(event->cut());
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
        wz = -0.5;
    }

   uint32_t visualid;
   if (this->link->VisualId("visual", visualid)){
       ignition::math::Pose3d pose;//(0,0,wz,0,0,0);
       if(this->link->VisualPose(visualid,pose)){
           pose.Pos().Z() = wz;
           if(this->link->SetVisualPose(visualid,pose)){
               std::cout << "w1";
            }
       }

   }

   this->model->SetScale(nscale,true);
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