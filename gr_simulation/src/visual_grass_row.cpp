#include <visual_grass_row.h>
#include <gtest/gtest.h>


using namespace gazebo;

VisualGrassRow::VisualGrassRow():is_cut(false), access_counter(0){
    // Initialize ros, if it has not already bee initialized.
}

VisualGrassRow::~VisualGrassRow(){
    std::cout << "KILLED?"<<std::endl;
    this->model.reset();
}

/*void VisualGrassRow::Init(){
    std::cout << "Init " << access_counter << std::endl;
    access_counter++;
    if (this->rosSub.getTopic().empty()){
        ROS_INFO("initialize subscriber");
        this->rosSub = this->nh->subscribe("test", 1, &VisualGrassRow::ros_cb, this);
        ROS_INFO("subscriber working");
        ros::spinOnce();
    }
    //std::cout << "SUbscribed to topic " << this->sub->GetTopic() << std::endl;
    //OnEvent();

}*/

//seems Load function runs twice when added manually on Gazebo server
//TODO review behavior when a world is loaded
void VisualGrassRow::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf){
    
    this->model = _parent;
    std::cout << "Load " << access_counter << std::endl;
    access_counter++;

    bool use_ros=false;

    if (_sdf->HasElement("use_ros")){
        use_ros = _sdf->Get<bool>("use_ros");
    }

    model_id = this->model->Name();//this->model->Name();      
    std::cout << "Model Name " << model_id << std::endl;
    auto start_idx = model_id.find("grassrow");
    auto end_idx = model_id.find("link")-2;
    model_id = model_id.substr(start_idx, end_idx);
    std::string topicName = "/" + model_id;
    std::cout << topicName << std::endl;

    if (!use_ros){
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init();//this->model->Name());    
        this->sub = this->node->Subscribe(topicName,&VisualGrassRow::OnRequest, this);
    }

    if (use_ros){
        if (!ros::isInitialized()){
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
        }
        this->nh = boost::make_shared<ros::NodeHandle>();
        //Subscriber
        ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Bool>(
            topicName,1,
            boost::bind(&VisualGrassRow::ros_cb, this, _1),
            ros::VoidPtr(), &this->rosQueue);    
        this->rosSub = this->nh->subscribe(so);
        // Spin up the queue helper thread.
        this->rosQueueThread = std::thread(std::bind(&VisualGrassRow::QueueThread, this));
    }
}

void VisualGrassRow::ros_cb(const std_msgs::BoolConstPtr &_msg){
    OnEvent(_msg->data);
}

void VisualGrassRow::OnRequest(GrassCutterRequestPtr &event){
    OnEvent(event->cut());
}


void VisualGrassRow::OnEvent(bool state){
    is_cut = state;
    this->model->SetVisible(is_cut);
    //this->model->Update();
}

void VisualGrassRow::QueueThread(){
    static const double timeout = 0.01;
    while (this->nh->ok()){
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}
