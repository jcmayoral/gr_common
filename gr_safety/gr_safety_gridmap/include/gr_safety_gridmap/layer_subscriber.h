#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>

//Based originally from the rosbag c++ implementation
//https://github.com/ros/ros_comm/tree/noetic-devel/tools/rosbag/src
//used as tutorial to get into the definition of a message.
namespace gr_safety_gridmap{
    static grid_map::GridMap gridmap;

    class LayerSubscriber{
        public: 
            void layerCB(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event);
            std::string getLayerId();
            void reset();
            void updateLayer(const geometry_msgs::PoseArray& poses, int behaviour);
            void updateLayer(const nav_msgs::Path& path, int behaviour);
            LayerSubscriber();
            LayerSubscriber(const LayerSubscriber& other);
            LayerSubscriber(std::string input, std::string id);

        private:
            boost::shared_ptr<ros::Subscriber> sub_;
            ros::NodeHandle nh_;  
            ros::Subscriber rsub_;  
            std::string id_;
    };
};
