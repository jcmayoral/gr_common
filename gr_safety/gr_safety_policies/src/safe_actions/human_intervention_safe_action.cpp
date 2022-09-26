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
    std_srvs::Trigger::Request srv_req;
    std_srvs::Trigger::Response srv_resp;
    ros::service::call("/gr_human_intervention/set", srv_req, srv_resp);
};

void HumanInterventionSafeAction::stop(){
    ROS_INFO("stop is same as execute in Human Intervention");
    std_srvs::Trigger::Request srv_req;
    std_srvs::Trigger::Response srv_resp;
    ros::service::call("/gr_human_intervention/set", srv_req, srv_resp);
};
