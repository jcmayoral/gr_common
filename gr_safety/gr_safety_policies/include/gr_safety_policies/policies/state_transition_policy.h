#ifndef HRI_POLICY_H
#define HRI_POLICY_H

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

    private:
      //get state
      //std::string state_;
      std::string last_state_;
      ros::Subscriber states_sub_;
      //manager includes all transactions informations
      //TransitionsManager* manager_;
      TransitionArray manager_;
      //This instantiate a single class
      boost::shared_ptr<ActionHelper<safety_core::SafeAction>> current_action_;
      pluginlib::ClassLoader<safety_core::SafeAction> action_loader_;

  };

};

#endif
