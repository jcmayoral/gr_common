#ifndef _GAZEBO_OBSTACLE_MOTIONPLANNER_HH_
#define _GAZEBO_OBSTACLE_MOTIONPLANNER_HH_

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <sbpl/headers.h>

//TODO
#include <boost/thread/mutex.hpp>

namespace gazebo{
    class MotionPlanner{
        public:
            MotionPlanner();
            MotionPlanner(MotionPlanner&& motionplanner){

            }
            void OnMsg(ConstPosePtr &_msg);
            void operator()(gazebo::transport::NodePtr node, std::string obstacleid);
            bool planPath();
            void ExecuteCommand();
            void run(gazebo::transport::NodePtr node, std::string obstacleid);
        private:
            transport::SubscriberPtr odom_sub_;
            transport::PublisherPtr vel_pub_;
            std::string obstacleid_;
            SBPLPlanner* planner_;
            EnvironmentNAVXYTHETALAT* env_;
            msgs::Vector3d current_pose_;
            std::vector<EnvNAVXYTHETALAT3Dpt_t> sbpl_path_;
            //this crashes inside the thread
            bool initialized_;
            double resolution_;
            int ncells_;
            boost::mutex mtx_;
    };
}

#endif 