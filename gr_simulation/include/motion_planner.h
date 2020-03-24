#ifndef _GAZEBO_OBSTACLE_MOTIONPLANNER_HH_
#define _GAZEBO_OBSTACLE_MOTIONPLANNER_HH_

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <sbpl/headers.h>
//TODO
#include <boost/thread/mutex.hpp>
#include <thread>

namespace gazebo{
    class MotionPlanner{
        public:
            MotionPlanner();
            MotionPlanner(MotionPlanner&& motionplanner){
                std::cerr <<  "Copy constructor " << std::endl;
                initialized_ = false;
                resolution_ = motionplanner.resolution_;
                ncells_ = motionplanner.ncells_;
                offset_ = motionplanner.offset_;
                error_ = motionplanner.error_;
                odom_sub_ = motionplanner.odom_sub_;
                vel_pub_ = motionplanner.vel_pub_;
                obstacleid_ = motionplanner.obstacleid_;
                planner_ = motionplanner.planner_;
                env_ = motionplanner.env_;
                current_pose_ = motionplanner.current_pose_;
            }
            void OnMsg(ConstPosePtr &_msg);
            void operator()(gazebo::transport::NodePtr node, std::string obstacleid);
            bool planPath(double goalx, double goaly, double goalyaw);
            void ExecuteCommand();
            bool run(gazebo::transport::NodePtr node, std::string obstacleid);
            void stop();
            void performMotion();

        private:
            transport::SubscriberPtr odom_sub_;
            transport::PublisherPtr vel_pub_;
            std::string obstacleid_;
            SBPLPlanner* planner_;
            EnvironmentNAVXYTHETALAT* env_;
            msgs::Vector3d current_pose_;
            std::vector<EnvNAVXYTHETALAT3Dpt_t> sbpl_path_;
            bool initialized_;
            double resolution_;
            int ncells_;
            double offset_;
            double error_;
            boost::mutex mtx_;
    };
}

#endif 