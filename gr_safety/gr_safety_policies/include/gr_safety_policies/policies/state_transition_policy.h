#ifndef STATE_POLICY_H
#define STATE_POLICY_H

#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <detection_msgs/BoundingBoxes.h>
#include <safety_msgs/StateTransition.h>

#include <safety_core/safe_action.h>
#include <safety_core/safe_policy.h>

#include <pluginlib/class_loader.h>

#include <boost/thread/recursive_mutex.hpp>
#include <boost/algorithm/string.hpp>

#include <gr_safety_policies/utils/action_helper.hpp>
#include <gr_safety_policies/utils/yaml-parser.hpp>
#include <gr_safety_policies/utils/matching_persons.hpp>

#include <boost/thread/mutex.hpp>
#include <pluginlib/class_loader.h>

//#include <dynamic_reconfigure/server.h>

namespace gr_safety_policies
{

  class StateTransitionPolicy : public safety_core::SafePolicy
  {
    public:

      StateTransitionPolicy();
      ~StateTransitionPolicy();

      void instantiateServices(ros::NodeHandle nh);
      bool checkPolicy();
      void doAction();
      void states_CB(const detection_msgs::BoundingBoxesConstPtr current_detections);
      void updateState(const ros::TimerEvent& event);
      void undoAction(std::string action);
      void updateState();

    private:
      std::string action_;
      bool update_;
      double clear_delay_;
      int riskier_id_;
      bool is_same_person_;
      bool last_person_in_borders_;

      std::string state_t_str_;
      std::string state_t1_str_;
      std::string last_executed_action_;

      TransitionInfo* action_info_;
      BoundingBoxInfo* bb_info_;
      //get state
      //std::string state_;
      ros::Subscriber states_sub_;
      RiskManager manager_;

      boost::shared_ptr<ActionHelper<safety_core::SafeAction>> current_action_;
      pluginlib::ClassLoader<safety_core::SafeAction> action_loader_;
      boost::mutex mtx_;

      ros::Timer timer_;
      ros::Time last_detection_time_;

      ros::Publisher fb_pub_;
      safety_msgs::StateTransition fb_msg_;

  };

};

#endif
