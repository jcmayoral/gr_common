#include "gr_safety_policies/policies/state_transition_policy.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

PLUGINLIB_EXPORT_CLASS(gr_safety_policies::StateTransitionPolicy,
                        safety_core::SafePolicy)
using namespace safety_core;


namespace gr_safety_policies
{
    StateTransitionPolicy::StateTransitionPolicy():
        last_state_("Unknown"),
        action_loader_("safety_core", "safety_core::SafeAction")
    {
        manager_ = parseFile("config/state_policy.yaml");

        //loadActionClasses();
        policy_.id_ = "STATE_TRANSITION_POLICY";
        //TODO remove action
        policy_.action_ =  -1;
        policy_.state_ = PolicyDescription::UNKNOWN;
        policy_.type_ = PolicyDescription::ACTIVE;
        ROS_INFO("State Transition Policy initialized");
    }

    StateTransitionPolicy::~StateTransitionPolicy(){

    }

    /*
    void StateTransitionPolicy::loadActionClasses(){
        action_classes_ = action_loader_.getDeclaredClasses();
    }
    */

   void StateTransitionPolicy::states_CB(detection_msgs::BoundingBoxesConstPtr current_detections){
    //Asssumes topic is just published when at least one person is detected
    if (current_detections->bounding_boxes.size()==0){
        ROS_ERROR("NO DETECTIONS");
        //Add timer or approach to handle personas getting out of the plane
        return;
    }
    int current_state = std::numeric_limits<int>::max();
    std::string current_state_str = "";

    for (auto it = current_detections->bounding_boxes.begin(); it!=current_detections->bounding_boxes.end();it++){
        //ROS_WARN_STREAM(it->Class << it->probability);
        ROS_WARN_STREAM(manager_.levels[it->Class]);
        if (manager_.levels[it->Class]< current_state){
            current_state = manager_.levels[it->Class];
            current_state_str = it->Class;
        }
    }
    ROS_INFO_STREAM(last_state_ << " " << current_state);
    ROS_INFO_STREAM(manager_.transition[last_state_][current_state_str].action);
    last_state_ = current_state_str;
   }

    void StateTransitionPolicy::instantiateServices(ros::NodeHandle nh){
        states_sub_ = nh.subscribe("/yolov5/detections", 1, &StateTransitionPolicy::states_CB, this);
    }

    bool StateTransitionPolicy::checkPolicy(){
        return true;//is_action_requested_;
    }

    void StateTransitionPolicy::suggestAction(){

    }


    /*
    void StateTransitionPolicy::instantiateRequestedAction(std::string desired_action){
        try{
             if(action_loader_.isClassAvailable(desired_action)){
                 ROS_INFO("Available Action... Loading");
                 current_action_ = action_loader_.createInstance(desired_action.c_str());
                 ROS_INFO("Created safe_action %s", desired_action.c_str());
                 is_action_requested_ = true;
             }
        }
         catch (const pluginlib::PluginlibException& ex){
             ROS_FATAL("Failed to create action %s", ex.what());
             exit(1);
        }

    }
    */
}
