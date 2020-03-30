#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
//Based originally from the rosbag c++ implementation
//https://github.com/ros/ros_comm/tree/noetic-devel/tools/rosbag/src
//used as tutorial to get into the definition of a message.
namespace gr_safety_gridmap{
    static grid_map::GridMap gridmap;
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
                    updateLayer();
                }

                std::string getLayerId(){
                    return id_;
                }

                void updateLayer(){
                    //boost::shared_ptr<grid_map::GridMap> pmap;
                    //path = nullptr;
                    std::cout << "updatelayer"<< id_ << std::endl;
                    //pmap = boost::static_pointer_cast<grid_map::GridMap>(map);
                    ROS_INFO_STREAM(gridmap.exists(id_));
                    gridmap.add(id_, 0);
                    //ROS_INFO_STREAM(*path);<d
                    grid_map::Position position;
                    grid_map::Index index;
                    int c = 0;
                    for (auto p : path.poses){
                        position(0) = p.pose.position.x;
                        position(1) = p.pose.position.y;
                        gridmap.getIndex(position, index);
                        ROS_INFO_STREAM(index);
                        gridmap.at(id_, index) = 0.1 *exp(-c);
                        c++;
                    }
                    message_received_ = false;
                }

                bool isMessageReceived(){
                    return message_received_;
                }

                LayerSubscriber(){

                 }
                
                LayerSubscriber(const LayerSubscriber& other){
                    id_ = other.id_;
                    rsub_ = other.rsub_;
                    message_received_ = other.message_received_;
                 }
                
                LayerSubscriber(std::string input, std::string id): id_(id), message_received_(false){
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
                T path;
                bool message_received_;
                std::string id_;
    };
};

//template <typename T>
//bool gr_safety_gridmap::LayerSubscriber<T>::message_received_ = false;
//template <typename T>
//T gr_safety_gridmap::LayerSubscriber<T>::path = T();