#include <visual_grass_row.h>
#include <gtest/gtest.h>


using namespace gazebo;

VisualGrassRow::VisualGrassRow():is_cut(false), access_counter(0){
    // Initialize ros, if it has not already bee initialized.
}

VisualGrassRow::~VisualGrassRow(){
    this->model.reset();
}

void VisualGrassRow::Init(){
    //CallBack for Gazebo
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->Name());

    //this->model->SetName("grassrow");
    model_id = this->model->Name();//this->model->Name();  
    
    std::cout << "Model Name " << model_id << std::endl;
    auto start_idx = model_id.find("grassrow");
    auto end_idx = model_id.find("link")-2;
    model_id = model_id.substr(start_idx, end_idx);
    std::string topicName = "~/" + model_id + "/event";
    std::cout << topicName << "  final topic" << std::endl;
    // Just output a message for now
    this->sub = this->node->Subscribe(topicName,&VisualGrassRow::OnRequest, this);

    std::cout << "Init " << access_counter << std::endl;
    access_counter++;
}

//seems Load function runs twice when added manually on Gazebo server
//TODO review behavior when a world is loaded
void VisualGrassRow::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf){
    this->model = _parent;  

    std::cout << "Load " << access_counter << std::endl;
    access_counter++;
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

}

void VisualGrassRow::OnRequest(ConstEmptyPtr &event){
    OnEvent();
}


void VisualGrassRow::OnEvent(){
    ignition::math::Vector3d new_scale;
    new_scale = this->model->Scale();
    std::cout << "SX" << new_scale.X() << std::endl;
    std::cout << "SY" << new_scale.Y() << std::endl;
    std::cout << "SZ " << new_scale.Z() << std::endl;

    new_scale.Z() = 1.0;

    if (is_cut){
        new_scale.Z() = 10.;
    }
    new_scale.Y() = 1.0;
    new_scale.X() = 1.0;
    this->model->SetScale(new_scale);
    //new_scale = this->model->Scale();
    //ignition::math::Pose3d update_pose = ignition::math::Pose3d(1,1,-1,0,0,0);
    //this->model->SetWorldPose(update_pose);
    is_cut = !is_cut;
}
