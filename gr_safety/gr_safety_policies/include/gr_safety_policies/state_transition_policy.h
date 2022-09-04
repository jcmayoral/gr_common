#ifndef HRI_POLICY_H
#define HRI_POLICY_H

#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>

#include <safety_core/safe_action.h>
#include <safety_core/safe_policy.h>

#include <pluginlib/class_loader.h>

#include <boost/thread/recursive_mutex.hpp>
#include <boost/algorithm/string.hpp>


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

    private:
      boost::shared_ptr<safety_core::SafeAction> current_action_;
      boost::recursive_mutex mutex;

      pluginlib::ClassLoader<safety_core::SafeAction> action_loader_;

  };

};

#endif
