#include <gr_safety_policies/safe_actions/human_intervention.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(gr_safety_policies::HumanInterventionSafeAction,safety_core::SafeAction);
using namespace gr_safety_policies;


HumanInterventionSafeAction::HumanInterventionSafeAction(){
    ROS_INFO("ServiceCallerSafeAction Safe Action");
    safety_id_ = 2;
};

HumanInterventionSafeAction::~HumanInterventionSafeAction(){
};

void HumanInterventionSafeAction::execute(){

};

void HumanInterventionSafeAction::stop(){
    //ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
};
