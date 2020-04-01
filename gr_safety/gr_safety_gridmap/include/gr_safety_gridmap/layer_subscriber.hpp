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
    class MainGrid{
        public:
        /*
            inline void lock(){
                std::cout << "lock"<< std::endl;
                mtx.lock();
            }
            inline void unlock(){
                std::cout << "unlock"<< std::endl;
                mtx.unlock();
            }
            */
            grid_map::GridMap gridmap;
            boost::mutex mtx;
    };

    static gr_safety_gridmap::MainGrid gridmap;

    //todo pass type geometry_msgs::PoseStamped without template
    class LayerSubscriber{
        public: 
            void layerCB(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event){
                //boost::mutex::scoped_lock lck(mt);
                boost::shared_ptr<topic_tools::ShapeShifter const> const &ssmsg = msg_event.getConstMessage();
                auto msginfo = msg_event.getConnectionHeader();
                std::string def = ssmsg->getMessageDefinition();
                
                std::string msgtype = msginfo["type"];
                //Check if path
                //TODO behaviour according to type
                
                try{
                    nav_msgs::Path path;
                    path = *ssmsg->instantiate<nav_msgs::Path>();
                    updateLayer(path, 0);
                    ROS_INFO_STREAM("IT is a path");
                }
                catch(...){
                }

                try{
                    geometry_msgs::PoseArray parr;
                    parr = *ssmsg->instantiate<geometry_msgs::PoseArray>();
                    updateLayer(parr, 1);
                }

                catch(...){
                    ROS_ERROR("WTF");
                }
                
            }

            std::string getLayerId(){
                return id_;
            }

            void reset(){
                //gridmap.lock();
                gridmap.gridmap.add(id_, 0);
                //gridmap.unlock();
            }

            void convert(geometry_msgs::Pose& in){
                tf2::doTransform(in, in, to_global_transform);
            }

            void updateLayer(const geometry_msgs::PoseArray& poses, int behaviour){
                //boost::shared_ptr<grid_map::GridMap> pmap;
                grid_map::Position position;
                grid_map::Index index;

                boost::mutex::scoped_lock lck(gridmap.mtx);
                {
                //gridmap.lock();
                reset();
                int c = 0; 
                float radius = 0.5;
                to_global_transform = tf_buffer_.lookupTransform("odom", "velodyne", ros::Time::now(), ros::Duration(0.1) );

                /*
                for (auto p : poses.poses){
                    convert(p);
                    position(0) = p.position.x;
                    position(1) = p.position.y;
                    gridmap.gridmap.getIndex(position, index);
                    //gridmap.gridmap.at(id_, index) = std::max(static_cast<double>(gridmap.gridmap.at(id_, index)),exp(-0.005*c));
                    
                    for (grid_map::CircleIterator iterator(gridmap.gridmap, position, radius);!iterator.isPastEnd(); ++iterator) {
                        gridmap.gridmap.at(id_, *iterator) = std::max(static_cast<double>(gridmap.gridmap.at(id_, index)),exp(-0.005*c));
                    }
                    c++;
                }
                */
                 for (auto p : poses.poses){
                    auto odompose = p;
                    auto aux = odompose;
                    generateCycle(aux,5);
                    /*
                    //time
                    for (int t=0; t< 5; t++){
                        //primitives
                        for (int i=0; i <9; i++){
                            generateMotion(odompose,i);
                            aux = odompose;
                            //to odom frame
                            convert(aux);
                            position(0) = aux.position.x;
                            position(1) = aux.position.y;
                            gridmap.gridmap.getIndex(position, index);
                            //gridmap.gridmap.at(id_, index) = std::max(static_cast<double>(gridmap.gridmap.at(id_, index)),exp(-0.005*c));
                            
                            for (grid_map::CircleIterator iterator(gridmap.gridmap, position, radius);!iterator.isPastEnd(); ++iterator) {
                                gridmap.gridmap.at(id_, *iterator) = std::max(static_cast<double>(gridmap.gridmap.at(id_, index)),exp(-0.5*t));
                            }
                        }
                        odompose = aux;
                    }
                    */
                }
                }
                //gridmap.unlock();
            }

            //TODO create MotionModelClass   
            void generateCycle(geometry_msgs::Pose& in, int depth){
                if (depth==0){
                    return;
                }

                depth--;
                generateCycle(in,depth);
                geometry_msgs::Pose aux;
                grid_map::Position position;
                grid_map::Index index;
                float radius = 0.25;

                 for (int i=0; i <9; i++){
                    generateMotion(in,i);
                    aux = in;
                    //to odom frame
                    convert(aux);
                    position(0) = aux.position.x;
                    position(1) = aux.position.y;
                    gridmap.gridmap.getIndex(position, index);
                    //gridmap.gridmap.at(id_, index) = std::max(static_cast<double>(gridmap.gridmap.at(id_, index)),exp(-0.005*c));
                    for (grid_map::CircleIterator iterator(gridmap.gridmap, position, radius);!iterator.isPastEnd(); ++iterator) {
                        gridmap.gridmap.at(id_, *iterator) = std::max(static_cast<double>(gridmap.gridmap.at(id_, index)),exp(-0.5*depth));
                    }
                 }
            }

            //TODO create MotionModelClass
            void generateMotion(geometry_msgs::Pose& in, int motion_type){
                auto distance = 0.5;
                switch(motion_type){
                    case 0:
                        break;
                    case 1:
                        in.position.x+=distance;
                        break;
                    case 2:
                        in.position.x-=distance;
                        break;
                    case 3:
                        in.position.y+=distance;
                        break;
                    case 4:
                        in.position.y-=distance;
                        break;
                    case 5:
                        in.position.x+=distance;
                        in.position.y+=distance;
                        break;
                    case 6:
                        in.position.x-=distance;
                        in.position.y-=distance;
                        break;
                    case 7:
                        in.position.x-=distance;
                        in.position.y+=distance;
                        break;
                    case 8:
                        in.position.x+=distance;
                        in.position.y-=distance;
                        break;
                }
            }

            void updateLayer(const nav_msgs::Path& path, int behaviour){
                //boost::shared_ptr<grid_map::GridMap> pmap;
                grid_map::Position position;
                grid_map::Index index;
                std::cout << "updateLayer" << id_ << std::endl;

                int c = 0;

                boost::mutex::scoped_lock lck(gridmap.mtx);
                {
                reset();
                //gridmap.lock();
                if (behaviour==1)
                    to_global_transform = tf_buffer_.lookupTransform("odom", "velodyne", ros::Time::now(), ros::Duration(0.1) );

                for (auto p : path.poses){
                    if (behaviour==1){
                        convert(p.pose);
                    }
                    position(0) = p.pose.position.x;
                    position(1) = p.pose.position.y;
                    gridmap.gridmap.getIndex(position, index);
                    gridmap.gridmap.at(id_, index) = std::max(static_cast<double>(gridmap.gridmap.at(id_, index)),exp(-0.005*c));
                    c++;
                }
                };
                //gridmap.unlock();
            }
            
            LayerSubscriber(const LayerSubscriber& other): tf2_listener_(tf_buffer_), nh_(){
                id_ = other.id_;
                topic_ = other.topic_;
                ops.topic = topic_;//"/" + input;//options_.rate_control_topic;
                ops.queue_size = 1;
                ops.md5sum = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
                ops.datatype = ros::message_traits::datatype<topic_tools::ShapeShifter>();
                ops.helper = boost::make_shared<ros::SubscriptionCallbackHelperT<
                        const ros::MessageEvent<topic_tools::ShapeShifter const> &> >(
                                boost::bind(&LayerSubscriber::layerCB, this, _1));
                rsub_ = nh_.subscribe(ops);

                //rsub_ = other.rsub_;
                //tf_buffer_ = other.tf_buffer_;
                //tf2_listener_ = other.tf2_listener_;
            }
            
            LayerSubscriber(std::string input, std::string id): id_(id),tf2_listener_(tf_buffer_), nh_(){
                ROS_INFO_STREAM(id_);
                topic_ = "/" + input;
                ops.topic = topic_;//"/" + input;//options_.rate_control_topic;
                ops.queue_size = 1;
                ops.md5sum = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
                ops.datatype = ros::message_traits::datatype<topic_tools::ShapeShifter>();
                ops.helper = boost::make_shared<ros::SubscriptionCallbackHelperT<
                        const ros::MessageEvent<topic_tools::ShapeShifter const> &> >(
                                boost::bind(&LayerSubscriber::layerCB, this, _1));
                rsub_ = nh_.subscribe(ops);
            }

        private:
            std::string topic_;
            boost::shared_ptr<ros::Subscriber> sub_;
            ros::NodeHandle nh_;  
            ros::Subscriber rsub_;  
            std::string id_;
            geometry_msgs::TransformStamped to_global_transform;
            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf2_listener_;
            boost::mutex mt;
            ros::SubscribeOptions ops;

    };
};


