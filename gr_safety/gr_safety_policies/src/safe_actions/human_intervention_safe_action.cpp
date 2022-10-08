#include <gr_safety_policies/safe_actions/human_intervention.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(gr_safety_policies::HumanInterventionSafeAction,safety_core::SafeAction);
using namespace gr_safety_policies;


HumanInterventionSafeAction::HumanInterventionSafeAction(){
    ROS_INFO("ServiceCallerSafeAction Safe Action");
    safety_id_ = 2;
    ros::NodeHandle nh;
    notification_pub_ = nh.advertise<std_msgs::Empty>("hri_requested", 1);
    while (notification_pub_.getNumSubscribers()<1){
        std::cout << "wait for subscribers" << std::endl;
        ros::Duration(0.1).sleep();
    } 
    std::cout << notification_pub_.getNumSubscribers() << std::endl;
};

HumanInterventionSafeAction::~HumanInterventionSafeAction(){
};

void HumanInterventionSafeAction::execute(){
    std_srvs::Trigger::Request srv_req;
    std_srvs::Trigger::Response srv_resp;
    ros::service::call("/gr_human_intervention/set", srv_req, srv_resp);

    std_msgs::Empty trig;
    notification_pub_.publish(trig);
    ros::Duration(0.1).sleep();

};

void HumanInterventionSafeAction::stop(){
    ROS_INFO("stop is same as execute in Human Intervention");
    std_srvs::Trigger::Request srv_req;
    std_srvs::Trigger::Response srv_resp;
    ros::service::call("/gr_human_intervention/set", srv_req, srv_resp);

    std_msgs::Empty trig;
    notification_pub_.publish(trig);
    ros::Duration(0.1).sleep();
};
