#include <gr_safety_policies/safe_actions/reconfigure_controller_safe_action.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(gr_safety_policies::ReconfigureControllerSafeAction,safety_core::SafeAction);
using namespace gr_safety_policies;


ReconfigureControllerSafeAction::ReconfigureControllerSafeAction(){
    ROS_INFO("Dynamic Reconfigure Safe Action");
    safety_id_ = 1;
};

ReconfigureControllerSafeAction::~ReconfigureControllerSafeAction(){
};

void ReconfigureControllerSafeAction::execute(){
    ROS_INFO("Execute Dynamic Reconfigure Safe Action");
    std_srvs::SetBool::Request srv_req;
    srv_req.data = true;
    std_srvs::SetBool::Response srv_resp;
    ros::service::call("/limit_safe_speed", srv_req, srv_resp);
};

void ReconfigureControllerSafeAction::stop(){
    ROS_INFO("Stopping setting to default params");
    std_srvs::SetBool::Request srv_req;
    srv_req.data = false;
    std_srvs::SetBool::Response srv_resp;
    ros::service::call("/limit_safe_speed", srv_req, srv_resp);
};
