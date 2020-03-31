#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

//Based originally from the rosbag c++ implementation
//https://github.com/ros/ros_comm/tree/noetic-devel/tools/rosbag/src
//used as tutorial to get into the definition of a message.
namespace gr_safety_gridmap{
    static grid_map::GridMap gridmap;

    //todo pass type geometry_msgs::PoseStamped without template
    class LayerSubscriber{
        public: 
            void layerCB(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event){
                boost::shared_ptr<topic_tools::ShapeShifter const> const &ssmsg = msg_event.getConstMessage();
                auto msginfo = msg_event.getConnectionHeader();
                std::string def = ssmsg->getMessageDefinition();
                
                std::string msgtype = msginfo["type"];
                ROS_INFO_STREAM(msgtype);

                //Check if path
                //TODO behaviour according to type
                
                try{
                    nav_msgs::Path path(*ssmsg->instantiate<nav_msgs::Path>());
                    reset();
                    updateLayer(path, 0);
                    ROS_INFO_STREAM("IT is a path");
                }
                catch(...){
                    auto parr = *ssmsg->instantiate<geometry_msgs::PoseArray>();
                    reset();
                    updateLayer(parr, 1);
                    ROS_INFO_STREAM("IT is an array");;
                }
            }

            std::string getLayerId(){
                return id_;
            }

            void reset(){
                gridmap.add(id_, 0);
            }

            void convert(geometry_msgs::Pose& in){
                tf2::doTransform(in, in, to_global_transform);
            }

            void updateLayer(const geometry_msgs::PoseArray& poses, int behaviour){
                //boost::shared_ptr<grid_map::GridMap> pmap;
                grid_map::Position position;
                grid_map::Index index;
                int c = 0;  
                for (auto p : poses.poses){
                    position(0) = p.position.x;
                    position(1) = p.position.y;
                    gridmap.getIndex(position, index);
                    gridmap.at(id_, index) = std::max(static_cast<double>(gridmap.at(id_, index)),exp(-0.005*c));
                    c++;
                }
            }

            void updateLayer(const nav_msgs::Path& path, int behaviour){
                //boost::shared_ptr<grid_map::GridMap> pmap;
                grid_map::Position position;
                grid_map::Index index;
                int c = 0;
                if (behaviour==1)
                    to_global_transform = tf_buffer_.lookupTransform("odom", "velodyne", ros::Time::now(), ros::Duration(0.5) );

                for (auto p : path.poses){
                    if (behaviour==1){
                        convert(p.pose);
                    }
                    position(0) = p.pose.position.x;
                    position(1) = p.pose.position.y;
                    std::cout << index << std::endl;
                    gridmap.getIndex(position, index);
                    gridmap.at(id_, index) = std::max(static_cast<double>(gridmap.at(id_, index)),exp(-0.005*c));
                    c++;
                }
            }

            LayerSubscriber():tf2_listener_(tf_buffer_){

            }
            
            LayerSubscriber(const LayerSubscriber& other): tf2_listener_(tf_buffer_){
                id_ = other.id_;
                rsub_ = other.rsub_;
                //tf_buffer_ = other.tf_buffer_;
                //tf2_listener_ = other.tf2_listener_;
            }
            
            LayerSubscriber(std::string input, std::string id): id_(id),tf2_listener_(tf_buffer_){
                ros::SubscribeOptions ops;
                ops.topic ="/" + input;//options_.rate_control_topic;
                ops.queue_size = 1;
                ops.md5sum = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
                ops.datatype = ros::message_traits::datatype<topic_tools::ShapeShifter>();
                ops.helper = boost::make_shared<ros::SubscriptionCallbackHelperT<
                        const ros::MessageEvent<topic_tools::ShapeShifter const> &> >(
                                boost::bind(&LayerSubscriber::layerCB, this, _1));
                rsub_ = nh_.subscribe(ops);
            }

        private:
            boost::shared_ptr<ros::Subscriber> sub_;
            ros::NodeHandle nh_;  
            ros::Subscriber rsub_;  
            std::string id_;
            geometry_msgs::TransformStamped to_global_transform;
            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf2_listener_;
    };
};


