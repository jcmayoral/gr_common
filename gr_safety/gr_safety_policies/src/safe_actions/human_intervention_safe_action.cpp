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
    std_srvs::SetBool::Request srv_req;
    std_srvs::SetBool::Response srv_resp;
    ros::service::call("/gr_human_intervention", srv_req, srv_resp);
};

void HumanInterventionSafeAction::stop(){
    std_srvs::SetBool::Request srv_req;
    std_srvs::SetBool::Response srv_resp;
    ros::service::call("/gr_human_intervention", srv_req, srv_resp);
};
