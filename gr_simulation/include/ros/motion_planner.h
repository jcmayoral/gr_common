#ifndef _ROS_OBSTACLE_MOTIONPLANNER_HH_
#define _ROS_OBSTACLE_MOTIONPLANNER_HH_

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <sbpl/headers.h>
//TODO
#include <boost/thread/mutex.hpp>
#include <thread>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>

namespace gazebo{
    class ROSMotionPlanner{
        public:
            ROSMotionPlanner(std::string primfile="my_mprim_test.mprim");
            ROSMotionPlanner(ROSMotionPlanner&& motionplanner){
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
                startpose_ = motionplanner.startpose_;
                current_goal_ = motionplanner.current_goal_;
                map_size_ = motionplanner.map_size_;
                primitives_filename_ = motionplanner.primitives_filename_;
            }
            void OnMsg(ConstPosePtr &_msg);
            bool operator()(gazebo::transport::NodePtr node, std::string obstacleid, double map_size, const msgs::Vector3d* goal = NULL);
            bool planPath();
            void ExecuteCommand();
            bool run(gazebo::transport::NodePtr node, std::string obstacleid, double mapsize);
            void stop();
            bool performMotion();
            void setupMap(std::string obstacleid, double mapsize);
            void setPrimitivesFilename(std::string filepath){
                primitives_filename_ = filepath;
            }

            void setResolution(double res){
                resolution_ = res;
            }

            std::vector<EnvNAVXYTHETALAT3Dpt_t> getSBPLPath(){
                return copy_sbpl_path_;
            }

            double getOffset(){
                return offset_;
            }

            void publishPath();

            ~ROSMotionPlanner(){
              std::cout << "AMOT"<< std::endl;
              //delete planner_;
              //delete env_;
              if (odom_sub_!=NULL){
                  odom_sub_->Unsubscribe();
              }
              rpub_.shutdown();
              rpub2_.shutdown();
              nh.shutdown();
              std::cout << "BMOT"<< std::endl;
            }

        private:
            transport::SubscriberPtr odom_sub_;
            transport::PublisherPtr vel_pub_;
            std::string obstacleid_;
            std::string primitives_filename_;
            SBPLPlanner* planner_;
            EnvironmentNAVXYTHETALAT* env_;
            msgs::Pose current_pose_;
            msgs::Pose startpose_;
            msgs::Vector3d current_goal_;
            std::vector<EnvNAVXYTHETALAT3Dpt_t> sbpl_path_;
            std::vector<EnvNAVXYTHETALAT3Dpt_t> copy_sbpl_path_;
            bool initialized_;
            double resolution_;
            int ncells_;
            double offset_;
            double error_;
            double map_size_;
            boost::mutex mtx_;
            ros::Publisher rpub_;
            ros::Publisher rpub2_;
            ros::NodeHandle nh;
            bool map_set;
    };
}

#endif
