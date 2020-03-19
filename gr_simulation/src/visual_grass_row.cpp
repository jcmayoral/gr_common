/*
NOTE WILL NOT WORK!!!!!!!!!!!!
Visual Plugin seems to be run once when object is instantiated on gazebo
Use Model Plugin instead
*/

#include <visual_grass_row.h>
#include <gtest/gtest.h>


using namespace gazebo;

VisualGrassRow::VisualGrassRow():is_cut(false), access_counter(0){
    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
        ros::init_options::NoSigintHandler);
    }
}

VisualGrassRow::~VisualGrassRow(){
    std::cout << "KILLED?"<<std::endl;
    this->model.reset();
}

void VisualGrassRow::Init(){
    std::cout << "Init " << access_counter << std::endl;
    access_counter++;

    /*
    if (this->rosSub.getTopic().empty()){
        ROS_INFO("initialize subscriber");
        this->rosSub = this->nh->subscribe("test", 1, &VisualGrassRow::ros_cb, this);
        ROS_INFO("subscriber working");
        ros::spinOnce();
    }
    */
    //std::cout << "SUbscribed to topic " << this->sub->GetTopic() << std::endl;
    //OnEvent();

}

//seems Load function runs twice when added manually on Gazebo server
//TODO review behavior when a world is loaded
void VisualGrassRow::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf){
    
    this->model = _parent;
    std::cout << "Load " << access_counter << std::endl;
    access_counter++;

    //CallBack for Gazebo
    //transport::init();
    //transport::run();
    //this->model->SetName("grassrow");
    model_id = this->model->Name();//this->model->Name();  
    
    std::cout << "Model Name " << model_id << std::endl;
    auto start_idx = model_id.find("grassrow");
    auto end_idx = model_id.find("link")-2;
    model_id = model_id.substr(start_idx, end_idx);
    std::string topicName = "/test";///" + model_id + "/event";
    std::cout << topicName << "  final topic" << std::endl;


    if (false){
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init("ggg");//this->model->Name());    
        this->sub = this->node->Subscribe("/test",&VisualGrassRow::OnRequest, this);
    }
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

    //this->nh.reset(new ros::NodeHandle("gazebo_client"));
    this->nh = boost::make_shared<ros::NodeHandle>();
    //Subscriber
    ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Bool>(
            topicName,1,
           boost::bind(&VisualGrassRow::ros_cb, this, _1),
         ros::VoidPtr(), &this->rosQueue);    
    //this->rosSub = this->nh.subscribe(topicName, 1, &GrassRow::OnRosMsg, this);
    this->rosSub = this->nh->subscribe(so);
    //update pose TODO fancy stuff
    // Spin up the queue helper thread.
    this->rosQueueThread = std::thread(std::bind(&VisualGrassRow::QueueThread, this));
    
    //sleep(2);
}

void VisualGrassRow::ros_cb(const std_msgs::BoolConstPtr &_msg){
    std::cout << "ros_cb" << std::endl;
    OnEvent();
}

void VisualGrassRow::OnRequest(GrassCutterRequestPtr &event){
    std::cout << "onRequest " << std::endl;
    OnEvent();
}


void VisualGrassRow::OnEvent(){
    ignition::math::Vector3d new_scale;
    //new_scale = this->model->Scale();
    std::cout << "SX" << new_scale.X() << std::endl;
    std::cout << "SY" << new_scale.Y() << std::endl;
    std::cout << "SZ " << new_scale.Z() << std::endl;
    //this->model->SetScale(new_scale);
    //new_scale = this->model->Scale();
    //ignition::math::Pose3d update_pose = ignition::math::Pose3d(1,1,-1,1.0,0,0);
    //this->model->SetWorldPose(update_pose);
    is_cut = this->model->GetVisible();
    is_cut = !is_cut;
    this->model->SetVisible(is_cut);
    this->model->Update();
}

void VisualGrassRow::QueueThread(){
    static const double timeout = 0.01;
    while (this->nh->ok()){
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}