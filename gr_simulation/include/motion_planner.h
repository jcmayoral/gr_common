#ifndef _GAZEBO_OBSTACLE_MOTIONPLANNER_HH_
#define _GAZEBO_OBSTACLE_MOTIONPLANNER_HH_

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo{
    class MotionPlanner{
        public:
            MotionPlanner(gazebo::transport::NodePtr node, std::string obstacleid);
            void OnMsg(ConstPosePtr &_msg);
        private:
            transport::SubscriberPtr odom_sub_;
            transport::SubscriberPtr vel_pub_;
            std::string obstacleid_;
    };
}

#endif 