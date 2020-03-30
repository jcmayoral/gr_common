#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
//Based originally from the rosbag c++ implementation
//https://github.com/ros/ros_comm/tree/noetic-devel/tools/rosbag/src
//used as tutorial to get into the definition of a message.
namespace gr_safety_gridmap{
    template<class T>
        class LayerSubscriber{
            public: 
                void plot(T t){
                    std::cout << "IN PLOT "<< std::endl;
                    ROS_INFO_STREAM(t);
                }

                void layerCB(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event){
                    boost::shared_ptr<topic_tools::ShapeShifter const> const &ssmsg = msg_event.getConstMessage();
                    std::string def = ssmsg->getMessageDefinition();

                    //TODO UPDATE Find a better way to do this
                    // Check the message definition.
                    std::istringstream f(def);
                    std::string s;
                    bool flag = false;
                    while(std::getline(f, s, '\n')) {
                        if (!s.empty() && s.find("#") != 0) {
                            //std::cout << s << std::endl;
                            //TODO FIND POSEArray or something like that
                            // Does not start with #, is not a comment.
                            if (s.find("Header ") == 0) {
                                flag = true;
                            }
                            break;
                        }
                    }

                    if (!flag) {
                        std::cout << std::endl << "WARNING: Rate control topic is bad, header is not first. MSG may be malformed and does not contain pose message." << std::endl;
                        return;
                    }
                    path = *ssmsg->instantiate<T>();
                    message_received_ = true; 
                }

                
                void updateLayer(const boost::shared_ptr<grid_map::GridMap>& map){
                    //boost::shared_ptr<grid_map::GridMap> pmap;
                    //path = nullptr;
                    std::cout << "updatelayer"<<std::endl;
                    //pmap = boost::static_pointer_cast<grid_map::GridMap>(map);
                    ROS_INFO_STREAM(map->exists(id_));
                    map->add(id_, 0);
                    //ROS_INFO_STREAM(*path);
                    grid_map::Position position;
                    ROS_INFO_STREAM(path);
                    message_received_ = false;

                }

                bool isMessageReceived(){
                    //boost::mutex::scoped_lock lock(mtx_);
                    //return message_received;
                    //For DEBUGGING
                   if(!message_received_){
                       std::cout << "Message has not been received " << message_received_ <<std::endl;
                       return false;
                   }
                   std::cout << "Message received " << message_received_ <<std::endl;
                   return true;//path->poses.size()>0 ? true : false;
                }
                
                LayerSubscriber(std::string id): id_(id){
                    ros::SubscribeOptions ops;
                    ops.topic ="/test";//options_.rate_control_topic;
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
                static T path;
                static bool message_received_;
                std::string id_;
    };
};

template <typename T>
bool gr_safety_gridmap::LayerSubscriber<T>::message_received_ = false;
template <typename T>
T gr_safety_gridmap::LayerSubscriber<T>::path = T();