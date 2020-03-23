#ifndef _GAZEBO_OBSTACLE_MOTIONPLANNER_HH_
#define _GAZEBO_OBSTACLE_MOTIONPLANNER_HH_

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <sbpl/headers.h>

namespace gazebo{
    class MotionPlanner{
        public:
            MotionPlanner();
            void OnMsg(ConstPosePtr &_msg);
            void operator()(gazebo::transport::NodePtr node, std::string obstacleid);
        private:
            transport::SubscriberPtr odom_sub_;
            transport::PublisherPtr vel_pub_;
            std::string obstacleid_;
            SBPLPlanner* planner_;
            EnvironmentNAVXYTHETALAT* env_;
    };
}

#endif 