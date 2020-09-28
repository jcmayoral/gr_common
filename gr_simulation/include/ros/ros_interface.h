#ifndef _ROS_INTERFACE_HH_
#define _ROS_INTERFACE_HH_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <gr_simulation/PersonMotionConfig.h>
#include <actionlib/client/simple_action_client.h>
#include <gr_action_msgs/SimMotionPlannerAction.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace gr_simulation{
    class ROSInterface{
        public:
            ROSInterface();
            virtual ~ROSInterface();
            void dyn_reconfigureCB(gr_simulation::PersonMotionConfig &config, uint32_t level);
        private:
            dynamic_reconfigure::Server<gr_simulation::PersonMotionConfig> dyn_server_;
            dynamic_reconfigure::Server<gr_simulation::PersonMotionConfig>::CallbackType dyn_server_cb_;
            boost::shared_ptr<actionlib::SimpleActionClient<gr_action_msgs::SimMotionPlannerAction>> ac_client_;
            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tf2_listener;

    };
};

#endif