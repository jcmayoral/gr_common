#include "gr_safety_policies/policies/state_transition_policy.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

PLUGINLIB_EXPORT_CLASS(gr_safety_policies::StateTransitionPolicy,
                        safety_core::SafePolicy)
using namespace safety_core;


namespace gr_safety_policies
{
    StateTransitionPolicy::StateTransitionPolicy():
        state_t1_str_("Unknown"),
        action_loader_("safety_core", "safety_core::SafeAction"),
        action_info_(new TransitionInfo()), state_t_str_("Unknown"),
        update_(false), last_detection_time_(ros::Time::now()), clear_delay_(10.0),
        last_executed_action_("")
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

    if (current_detections->bounding_boxes.size()==0){
        //ROS_ERROR("NO DETECTIONS");
        return;
    }

    int state_t = 10000;//manager_.levels[state_t_str_];
    //If not riskier state detected not update transition
    int state_t1 = manager_.levels[state_t1_str_];

    for (auto it = current_detections->bounding_boxes.begin(); it!=current_detections->bounding_boxes.end();it++){
        // ROS_WARN_STREAM(it->Class << " " << it->probability);
        //If risk state is higher than actual and last_state is different to state in t-1
        if (manager_.levels[it->Class]< state_t){
            state_t = manager_.levels[it->Class];
            state_t_str_ = it->Class;
        }
        last_detection_time_ = ros::Time::now();
    }


    if (state_t == state_t1 || state_t == 10000){
        /*
        if (current_detections->bounding_boxes.size()>0){
            ROS_ERROR_STREAM("DETECTIONS but no change.. state " << state_t_str_);
        }   
        else{
            ROS_WARN("No detection NO change");
        }
        */
        return;
    }

    *action_info_ = manager_.transition[state_t1_str_][state_t_str_];
   }

    void StateTransitionPolicy::instantiateServices(ros::NodeHandle nh){
        states_sub_ = nh.subscribe("/yolov5/detections", -1, &StateTransitionPolicy::states_CB, this);
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

    void StateTransitionPolicy::doAction(){
        //std::scoped_lock lock(mtx_);
        //ROS_INFO_STREAM("DO ACTION " << action_info_->action);
        ROS_INFO_STREAM("transition " << state_t1_str_ << " to "<< state_t_str_);

        if(action_loader_.isClassAvailable(action_info_->action)){
            last_executed_action_ = action_info_->action;
            boost::shared_ptr<safety_core::SafeAction> action;
            ROS_ERROR("do action");
            action = action_loader_.createInstance(action_info_->action);
            if (action_info_->negate){
                ROS_INFO("negate action");
                action->stop();
            }
            else{
                ROS_INFO("Execute action");
                action->execute();
            }
            policy_.action_ = action->getSafetyID();
        }
        update_ = true;
        updateState();
        policy_.state_ = PolicyDescription::UNSAFE;
    }

    void StateTransitionPolicy::updateState(const ros::TimerEvent& event){
        double transcurred_time = (ros::Time::now() - last_detection_time_).toSec();//seconds

        //Undo Action
        if (transcurred_time >= clear_delay_){
            undoAction();
            updateState();
        }
        
        //If action is executed
        if (!update_){
            return;
        }
    }


    void StateTransitionPolicy::updateState(){
        ROS_INFO("Update Info");
        //std::scoped_lock lock(mtx_);
        //state_t_ = std::numeric_limits<int>::max();
        action_info_ = new TransitionInfo();
        state_t1_str_= state_t_str_;
    }

    void StateTransitionPolicy::undoAction(){
        //std::scoped_lock lock(mtx_);f
        ROS_INFO_STREAM("undo action " << last_executed_action_);
        //last_executed_action
        if(action_loader_.isClassAvailable(last_executed_action_)){
            boost::shared_ptr<safety_core::SafeAction> action;
            action = action_loader_.createInstance(last_executed_action_);
            if (!action_info_->negate){
                ROS_ERROR("Undo negate action");
                action->stop();
            }
            else{
                ROS_ERROR("Undo Execute action");
                action->execute();
            }
            policy_.action_ = -1;
        }
        policy_.state_ = PolicyDescription::SAFE;
        last_executed_action_ = "";
        last_detection_time_ = ros::Time::now();
        update_ = true;
        state_t_str_ = "Unknown";
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
