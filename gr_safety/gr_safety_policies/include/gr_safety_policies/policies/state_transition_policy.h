#ifndef STATE_POLICY_H
#define STATE_POLICY_H

#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <detection_msgs/BoundingBoxes.h>

#include <safety_core/safe_action.h>
#include <safety_core/safe_policy.h>

#include <pluginlib/class_loader.h>

#include <boost/thread/recursive_mutex.hpp>
#include <boost/algorithm/string.hpp>

#include <gr_safety_policies/utils/action_helper.hpp>
#include <gr_safety_policies/utils/yaml-parser.hpp>

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
      void suggestAction();
      void states_CB(const detection_msgs::BoundingBoxesConstPtr current_detections);
      void updateState(const ros::TimerEvent& event);
      void clearState(const ros::TimerEvent& event);

    private:
      std::string action_;
      bool update_;
      int current_state_;
      double clear_delay_;

      std::string current_state_str_;
      TransitionInfo* action_info_;
      //get state
      //std::string state_;
      std::string last_state_str_;
      ros::Subscriber states_sub_;
      //manager includes all transactions informations
      //TransitionsManager* manager_;
      RiskManager manager_;
      //This instantiate a single class
      boost::shared_ptr<ActionHelper<safety_core::SafeAction>> current_action_;
      pluginlib::ClassLoader<safety_core::SafeAction> action_loader_;
      boost::mutex mtx_;

      ros::Timer timer_;
      ros::Time last_detection_time_;

  };

};

#endif