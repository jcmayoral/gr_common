#include <ros/ros.h>
//#include <topic_tools/shape_shifter.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/Path.h>
#include <string>
#include<iostream>

//Based originally from the rosbag c++ implementation
//https://github.com/ros/ros_comm/tree/noetic-devel/tools/rosbag/src
//used as tutorial to get into the definition of a message.
namespace gr_safety_gridmap{
    //static grid_map::GridMap cmap_;
    class LayerSubscriber{
        public: 
            void layerCB(const nav_msgs::Path::ConstPtr& msg_event);
            //void templatedlayerCB(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event);
            void updateLayer(nav_msgs::Path path);            
            std::string getLayerId();            
            LayerSubscriber(const std::string& id);
            ~LayerSubscriber(){
                std::cout << "destr"<<std::endl;
            }

            LayerSubscriber(const LayerSubscriber& other){
                 std::cout << "copy constructor " << other.layer_name_ << std::endl;
                 layer_name_ = other.layer_name_.c_str();
                                  std::cout << "copy constructor2 " << layer_name_ << std::endl;

                 rsub_ = other.rsub_;
                 nh_ = other.nh_;
             }

            LayerSubscriber(){
                layer_name_ = "fake";
                std::cout << "wrong constr subscriber to /test"<< layer_name_.c_str() << std::endl;
                 rsub_=nh_.subscribe("/test",1,&LayerSubscriber::layerCB, this);
            }
            grid_map::GridMap cmap_;
            std::string layer_name_;

        private:
            boost::shared_ptr<ros::Subscriber> sub_;
            ros::NodeHandle nh_;  
            ros::Subscriber rsub_;  
            bool message_received_;
            //grid_map::GridMap cmap_;
    };
};

//template <typename T>
//bool gr_safety_gridmap::LayerSubscriber<T>::message_received_ = false;
//template <typename T>
//T gr_safety_gridmap::LayerSubscriber<T>::path = T();

//template <typename T>
//T gr_safety_gridmap::LayerSubscriber<T>::cmap_ = grid_map::GridMap();