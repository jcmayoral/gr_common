#include <gr_safety_policies/policies/state_transition_policy.h>

using namespace gr_safety_policies;

int main(int argc, char** argv){
    ros::init(argc, argv, "state_transition_policy_node");
    ros::NodeHandle nh;
    StateTransitionPolicy* policy = new StateTransitionPolicy();
    policy->instantiateServices(nh);

    while(ros::ok()){
        if(policy->checkPolicy()){
            policy->reportState();
        }
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    return 1;
}
