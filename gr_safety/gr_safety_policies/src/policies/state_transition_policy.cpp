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
        update_(false), last_detection_time_(ros::Time::now()), clear_delay_(1.0),
        last_executed_action_(""), riskier_id_{std::numeric_limits<int>::max()},
        is_same_person_{false}, last_person_in_borders_{false}
    {
        bb_info_ = new BoundingBoxInfo();
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

   void StateTransitionPolicy::states_CB(const detection_msgs::BoundingBoxesConstPtr current_detections){
    std::scoped_lock lock(mtx_);

    if (current_detections->bounding_boxes.size()==0){
        //ROS_ERROR("NO DETECTIONS");
        return;
    }


    //If not riskier state detected not update transition
    int msg_riskier_state = 1000;
    std::string msg_state_str = "Unknown";

    detection_msgs::BoundingBox riskier_bb;
    double highest_iou = 0.0;

    //Get Most Dangerous from the msg
    for (auto it = current_detections->bounding_boxes.begin(); it!=current_detections->bounding_boxes.end();it++){
        //If risk state is lowe than actual
        if (manager_.levels[it->Class]<= manager_.levels[msg_state_str]){
            //ROS_INFO_STREAM("This person Class " << it->Class << " Current State in Msg " << msg_state_str);
            double iou = comparePersons(bb_info_, &(*it));
            if (iou<highest_iou){
                continue;
            }
            msg_riskier_state = manager_.levels[it->Class];
            //local iou
            highest_iou = iou;
            //strings
            msg_state_str = it->Class;
            //Bounding Box
            riskier_bb = std::move(*(it));
        }
        //update time
        last_detection_time_ = ros::Time::now();
    }

    //FEEDBACK
    fb_msg_.current = msg_state_str;
    fb_msg_.previous = state_t1_str_;
    fb_pub_.publish(fb_msg_);

    //Compare with timed window
    if(msg_riskier_state < riskier_id_){
        double iou = comparePersons(bb_info_, &riskier_bb);
        if(iou< 0.4){
            is_same_person_ = false;
            ROS_ERROR("different person from previous iteration");
        }
        else {
            is_same_person_ = true;
        }
        ROS_ERROR_STREAM("IOU "<< iou << "same person "<< is_same_person_);

        //new Coordinate from riskier person
        bb_info_->updateObject(&riskier_bb);

        //ROS_ERROR_STREAM("UPDATED " << bb_info_ << " IOU "<< iou << " is_same_person "<< is_same_person_);
        state_t_str_ = msg_state_str;
        *action_info_ = manager_.transition[state_t1_str_][state_t_str_];
        riskier_id_ = manager_.levels[msg_state_str];
    }

   }

    void StateTransitionPolicy::instantiateServices(ros::NodeHandle nh){
        fb_pub_ = nh.advertise<safety_msgs::StateTransition>("~/feedback",10);
        states_sub_ = nh.subscribe("/yolov5/detections", -1, &StateTransitionPolicy::states_CB, this);
        timer_ = nh.createTimer(ros::Duration(0.05), &StateTransitionPolicy::updateState,this);
    }

    bool StateTransitionPolicy::checkPolicy(){
        //Restart
        riskier_id_ = std::numeric_limits<int>::max();

        bool policy_state = false;
        //Check exceptionms
        if(action_info_->action != "None"){
            //Just testing reasons
            std::cout << is_same_person_ << last_person_in_borders_ << std::endl;
            if (!is_same_person_ && last_person_in_borders_){
                ROS_ERROR("Previous person detected in Borders has not detected...assuming all is correct");
                policy_state = false;
                //updateState();
                //last_person_in_borders_ = false;
            }
            else{
                policy_state = true;
            }
        }

        float centroidx = bb_info_->centroid_x/640.0;
        //std::cout << "centroid " << centroidx << std::endl;

         //Update flag
        /*
        if (0.15>centroidx>0 || 1.0>centroidx> 0.85){
            last_person_in_borders_ = true;
        }
        else{
            ROS_WARN("Person not in borders");
            last_person_in_borders_ = false;
        }
        */
        return policy_state;
    }

    void StateTransitionPolicy::doAction(){
        std::lock_guard lock(mtx_);
        //ROS_INFO_STREAM("DO ACTION " << action_info_->action);
        ROS_INFO_STREAM("transition " << state_t1_str_ << " to "<< state_t_str_ << " action name " << action_info_->action);

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
       std::lock_guard lock(mtx_);
       double transcurred_time = (ros::Time::now() - last_detection_time_).toSec();//seconds
       //Undo Action
       if (transcurred_time >= clear_delay_){
           undoAction("slow_action");
           undoAction("stop_action");
           //undoAction("human_intervention");

           updateState();
       }
       //bb_info_ = new BoundingBoxInfo();

    }


    void StateTransitionPolicy::updateState(){
        ROS_INFO("Update Info");
        // lock(mtx_);
        //state_t_ = std::numeric_limits<int>::max();
        action_info_ = new TransitionInfo();
        state_t1_str_= state_t_str_;
        riskier_id_ = std::numeric_limits<int>::max();
        //bb_info_ = new BoundingBoxInfo();
    }

    void StateTransitionPolicy::undoAction(std::string action_name){
        //std::scoped_lock lock(mtx_);f
        //last_executed_action
        if(action_loader_.isClassAvailable(action_name)){
            boost::shared_ptr<safety_core::SafeAction> action;
            action = action_loader_.createInstance(action_name);
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
