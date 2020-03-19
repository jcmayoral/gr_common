#include<dynamic_object_gazebo_interface.h>

using namespace gazebo;

GazeboDynamicObject::GazeboDynamicObject() {
}

void GazeboDynamicObject::SetAngVelocity(const double &_vel){
    ang_velocity = _vel;
    this->link->SetAngularVel(ignition::math::Vector3<double>(0.0,0.0,ang_velocity));		 
}

void GazeboDynamicObject::SetLinearVelocityX(const double &_vel){
    lin_velx = _vel;
    this->link->SetLinearVel(ignition::math::Vector3<double>(lin_velx,lin_vely,0.0));
}

void GazeboDynamicObject::SetLinearVelocityY(const double &_vel){
    lin_vely = _vel;
    this->link->SetLinearVel(ignition::math::Vector3<double>(lin_velx,lin_vely,0.0));
}

void GazeboDynamicObject::OnMsg(ConstVector3dPtr &_msg){
    this->SetLinearVelocityX(_msg->x());
    this->SetLinearVelocityY(_msg->y());
    this->SetAngVelocity(_msg->z());
    //current_pose = this->model->WorldPose();
    std::cout <<"X "<< current_pose.Pos().X();
}

void GazeboDynamicObject::OnUpdate(){
      // Apply a small linear velocity to the model.
      current_pose = this->model->WorldPose();
      this->pub->Publish(gazebo::msgs::Convert(current_pose));
    }


void GazeboDynamicObject::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
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
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->Name());

    std::string pubtopicName = "/" + this->model->GetName() + "/world_odom";
    this -> pub = this->node->Advertise<msgs::Pose>(pubtopicName);

    std::string subtopicName = "/" + this->model->GetName() + "/vel_cmd";
    this->sub = this->node->Subscribe(subtopicName,&GazeboDynamicObject::OnMsg, this);

     this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&GazeboDynamicObject::OnUpdate, this));

}