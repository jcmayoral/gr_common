#include "gr_safety_policies/policies/state_transition_policy.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

PLUGINLIB_EXPORT_CLASS(gr_safety_policies::StateTransitionPolicy,
                        safety_core::SafePolicy)
using namespace safety_core;


namespace gr_safety_policies
{
    StateTransitionPolicy::StateTransitionPolicy():
        last_state_str_("Unknown"),
        current_state_(std::numeric_limits<int>::max()),
        action_loader_("safety_core", "safety_core::SafeAction"),
        action_info_(new TransitionInfo()), current_state_str_("Unknown"),
        update_(false), last_detection_time_(ros::Time::now()), clear_delay_(5.0)
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
    std::scoped_lock lock(mtx_);
    //ROS_INFO("NEW MESSAGE");
    //ROS_ERROR_STREAM("current state " << current_state_str_);
    if (current_detections->bounding_boxes.size()==0){
        //ROS_ERROR("NO DETECTIONS");
        return;
    }
    bool changed = false;

    for (auto it = current_detections->bounding_boxes.begin(); it!=current_detections->bounding_boxes.end();it++){
        // ROS_WARN_STREAM(it->Class << " " << it->probability);
        //If risk state is higher than actual and last_state is different to detected
        if (manager_.levels[it->Class]<current_state_ && it->Class.compare(last_state_str_)!=0){
            changed = true;
            current_state_ = manager_.levels[it->Class];
            current_state_str_ = it->Class;
        }
        last_detection_time_ = ros::Time::now();
    }

    /*
    ROS_INFO_STREAM(last_state_ << " " << current_state);
    */
    if (!changed){
        return;
    }
    *action_info_ = manager_.transition[last_state_str_][current_state_str_];
   }

    void StateTransitionPolicy::instantiateServices(ros::NodeHandle nh){
        states_sub_ = nh.subscribe("/yolov5/detections", 10, &StateTransitionPolicy::states_CB, this);
        timer_ = nh.createTimer(ros::Duration(0.05), &StateTransitionPolicy::updateState,this);
    }

    bool StateTransitionPolicy::checkPolicy(){
        //Restart

        if(action_info_->action != "None"){
            return true;
        }
        else{
            return false;
        }
    }

    void StateTransitionPolicy::suggestAction(){
        std::scoped_lock lock(mtx_);
        ROS_INFO_STREAM("MY ACTION " << action_info_->action);
        ROS_INFO_STREAM("transition " << last_state_str_ << " to "<< current_state_str_);

        if(action_loader_.isClassAvailable(action_info_->action)){
            boost::shared_ptr<safety_core::SafeAction> action;
            ROS_ERROR("calling action");
            action = action_loader_.createInstance(action_info_->action);
            if (action_info_->negate){
                ROS_INFO("negate action");
                action->stop();
            }
            else{
                ROS_INFO("Execute action");
                action->execute();
            }
        }
        update_ = true;

    }

    void StateTransitionPolicy::updateState(const ros::TimerEvent& event){
        double execution_time = (ros::Time::now() - last_detection_time_).toSec();//seconds
        ROS_INFO_STREAM("update action " <<action_info_->action.c_str() << " last state "<< current_state_str_);

        ROS_INFO_STREAM("clear "<< execution_time);
        if (execution_time >= clear_delay_){
            clearState();
        }
        
        if (!update_){
            return;
        }
        ROS_WARN("Restart studdd");
        std::scoped_lock lock(mtx_);
        current_state_ = std::numeric_limits<int>::max();
        action_info_ = new TransitionInfo();
        last_state_str_= current_state_str_;
    }

    void StateTransitionPolicy::clearState(){
        //std::scoped_lock lock(mtx_);
        ROS_INFO_STREAM("clear action " <<action_info_->action.c_str());
        if(action_loader_.isClassAvailable(action_info_->action)){
            boost::shared_ptr<safety_core::SafeAction> action;
            ROS_ERROR("calling action");
            action = action_loader_.createInstance(action_info_->action);
            if (!action_info_->negate){
                ROS_INFO("Cancel negate action");
                action->stop();
            }
            else{
                ROS_INFO("Cancel Execute action");
                action->execute();
            }
        }

        last_detection_time_ = ros::Time::now();
        update_ = true;
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
