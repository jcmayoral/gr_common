#include "gr_safety_policies/policies/speech_policy.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

/*PLUGINLIB_DECLARE_CLASS(gr_safety_policies, SpeechPolicy,
                        gr_safety_policies::SpeechPolicy,
                        safety_core::SafePolicy)
*/
PLUGINLIB_EXPORT_CLASS(gr_safety_policies::SpeechPolicy,
                        safety_core::SafePolicy)
using namespace safety_core;


namespace gr_safety_policies
{
    SpeechPolicy::SpeechPolicy(): action_loader_("safety_core", "safety_core::SafeAction"),
                            is_action_executed_(false), is_stop_requested_(false),
                            is_action_requested_(false), speech_enabler_command_("continue"){
        loadActionClasses();
        policy_.id_ = "Speech_POLICY";
        policy_.action_ =  -1;
        policy_.state_ = PolicyDescription::UNKNOWN;
        policy_.type_ = PolicyDescription::ACTIVE;
        ROS_INFO("Speech Policy initialized");
    }

    SpeechPolicy::~SpeechPolicy(){

    }

    void SpeechPolicy::loadActionClasses(){
        action_classes_ = action_loader_.getDeclaredClasses();
    }

    void SpeechPolicy::instantiateServices(ros::NodeHandle nh){
        command_sub_ = nh.subscribe("commands", 1, &SpeechPolicy::commands_CB, this);
    }

    bool SpeechPolicy::checkPolicy(){
        return is_action_requested_;
    }

    void SpeechPolicy::doAction(){
        if (!is_action_executed_){
            current_action_->execute();
            policy_.state_ = PolicyDescription::UNSAFE;
            is_action_executed_ = true;
        }

        if (is_stop_requested_){
            current_action_->stop();
            policy_.state_ = PolicyDescription::SAFE;
            is_action_executed_ = false;
            is_action_requested_ = false;
            is_stop_requested_ = false;
        }

    }

    void SpeechPolicy::commands_CB(const std_msgs::String::ConstPtr& command){
        if (command->data.empty()){
          ROS_WARN("Empty command string...ignoring");
          return;
        }


        if (is_action_executed_){
            if (boost::contains(speech_enabler_command_, command->data.c_str())){
                is_stop_requested_ =true;
                ROS_INFO("ENABLE NORMAL PERFORMANCE");
                return;
            }
        }


        for (int i=0; i < action_classes_.size(); i++){
            if (boost::contains(action_classes_[i], command->data.c_str())){
            //if (boost::contains("no", command->data.c_str())){
                instantiateRequestedAction(action_classes_[i]);
                return;
            }
        }
    }

    void SpeechPolicy::instantiateRequestedAction(std::string desired_action){
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
}
