#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <safety_msgs/FoundObjectsArray.h>
#include <nav_msgs/Path.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include <gr_safety_gridmap/main_gridmap.hpp>

//Based originally from the rosbag c++ implementation
//https://github.com/ros/ros_comm/tree/noetic-devel/tools/rosbag/src
//used as tutorial to get into the definition of a message.
extern gr_safety_gridmap::MainGrid gridmap;

namespace gr_safety_gridmap{
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
                    //local gridmap -> 1
                    updateLayer(path, is_local_);
                    return;
                }
                catch(...){
                }

                try{
                    geometry_msgs::PoseArray parr;
                    parr = *ssmsg->instantiate<geometry_msgs::PoseArray>();
                    updateLayer(parr, 1);
                    return;
                }

                catch(...){
                }

                try{
                    safety_msgs::FoundObjectsArray parr;
                    parr = *ssmsg->instantiate<safety_msgs::FoundObjectsArray>();
                    updateLayer(parr, 1);
                    return;
                }

                catch(...){
                }
                
            }

            void addLayerTuple(std::string person_id){
                gridmap.gridmap.add(person_id, 0.0);
            }

            void convert(geometry_msgs::Pose& in){
                tf2::doTransform(in, in, to_global_transform);
            }

            void updateLayer(const safety_msgs::FoundObjectsArray& poses, int behaviour){
                //boost::mutex::scoped_lock lc(mt);
                //while (!gridmap.isNewDataAvailable());
                grid_map::Position position;
                grid_map::Index index;

                float radius = 0.5;
                to_global_transform = tf_buffer_.lookupTransform(map_frame_, poses.header.frame_id, ros::Time::now(), ros::Duration(0.1) );

                boost::mutex::scoped_lock lck(gridmap.mtx);{
                fb_msgs_.header.frame_id = map_frame_;
                fb_msgs_.poses.clear();
                //index 0 reserved to robot
                for (auto o : poses.objects){
                    addLayerTuple(o.object_id);
                    gridmap.update_times_[o.object_id] = ros::Time::now();                        
                    auto odompose = o.pose;
                    auto aux = odompose;
                    //move search_depth_ to motion model class
                    generateCycle(aux,search_depth_, o.object_id);
                }
                rpub_.publish(fb_msgs_);
                //gridmap.setDataFlag(true);
                };
            }

            void updateLayer(const geometry_msgs::PoseArray& poses, int behaviour){
                grid_map::Position position;
                grid_map::Index index;

                int c = 0; 
                float radius = 0.5;
                to_global_transform = tf_buffer_.lookupTransform(map_frame_, poses.header.frame_id, ros::Time::now(), ros::Duration(0.1) );

                boost::mutex::scoped_lock lck(gridmap.mtx);
                if (poses.poses.size()==0){
                    ROS_WARN_STREAM("No obstacle detected -> potential bug or implementation for memory");
                }
                //index 0 reserved to robot
                int person = 1;
                for (int i=1; i<poses.poses.size()+1;i++)
                    addLayerTuple(std::to_string(i));

                for (auto p : poses.poses){
                    auto odompose = p;
                    auto aux = odompose;
                    //move search_depth_ to motion model class
                    generateCycle(aux,search_depth_, std::to_string(person));
                    person++; //this can be calculated by std::distance
                }
                //gridmap.setDataFlag(true);
            }

            void generateCycle(geometry_msgs::Pose in, int depth, std::string layer){
                if (depth == 0){
                    return;
                }

                //generateCycle(in,depth);
                geometry_msgs::Pose aux, aux2;
                grid_map::Position position;
                grid_map::Index index;
                float radius = 1.0;
                aux = in;
                //MotionModel class TODO
                double costs[9] ={1.0,0.25,0.25,0.25,0.5,0.5,0.5,0.05,0.05};
                int nprimitives = 4;
                auto norm = search_depth_*nprimitives*exp(-0.1*(search_depth_-depth));

                for (int i=0; i <nprimitives; i++){
                    aux2 = generateMotion(in,i);
                    generateCycle(aux2,depth-1,layer);
                    //to mapframe
                    convert(aux2);
                    position(0) = aux2.position.x;
                    position(1) = aux2.position.y;
                    bool validindex = gridmap.gridmap.getIndex(position, index);
                    if (!validindex){
                        ROS_WARN_STREAM("index not valid"<<index<< map_frame_);
                        continue;
                    }
                    fb_msgs_.poses.push_back(aux2);
                    auto val = (search_depth_-depth)* exp(-0.1*(search_depth_-depth))*costs[i];
                    val /=norm;
                    gridmap.gridmap.at(layer, index) += val;//log(prob/(1-prob));
                }
            }

            //TODO create MotionModelClass
            geometry_msgs::Pose generateMotion(const geometry_msgs::Pose in, int motion_type){
                //this motion is a hack... motion on the sensor frame not the relative path
                geometry_msgs::Pose out;
                out = in;

                //double dt = (current_time - last_time).toSec();
                auto dt = 1;
                double vx = 0;
                double vy = 0;
                auto vth = 0.2;


                //x += delta_x;
                //y += delta_y;
                auto th = tf2::getYaw(in.orientation);
                double delta_th = vth * dt;

                switch(motion_type){
                    case 0:
                        //th += delta_th;
                        vx = resolution_;
                        break;
                    case 1:
                        vx = resolution_;
                        th += delta_th;
                        break;
                    case 2:
                        vx = resolution_;
                        th -= delta_th;
                        break;
                    case 3:
                        //out = in;
                        break;
                    case 4:
                        vx = -resolution_;
                        th -= delta_th;
                        break;
                    case 5:
                        vx = -resolution_;
                        th += delta_th;
                        break;
                    case 6:
                        vx = -resolution_;
                        //vy = 1;
                        //th -= delta_th;
                        break;
                    case 7:
                        //out.position.x = in.position.x-resolution_;
                        //out.position.y = in.position.y+resolution
                        th += delta_th;
                        break;
                    case 8:
                        //out.position.x = in.position.x+resolution_;
                        //out.position.y = in.position.y-resolution_;
                        th -= delta_th;
                        break;
                }

                tf2::Quaternion quat_tf;
                double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
                double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
                out.position.x = in.position.x+delta_x;
                out.position.y = in.position.y+delta_y;
                quat_tf.setEuler(0,0,th);
                //std::cout << th << " normalize?" << std::endl;
                //tf2::fromMsg(in.orientation, quat_tf);
                out.orientation = tf2::toMsg(quat_tf.normalize());
                return out;
            }

            void updateLayer(const nav_msgs::Path& path, int behaviour){
                //boost::shared_ptr<grid_map::GridMap> pmap;
                boost::mutex::scoped_lock lck(gridmap.mtx);
                grid_map::Position position;
                grid_map::Index index;
                //std::cout << "updateLayer" << id_ << std::endl;
                //std::cout << gridmap.id << "PATH OK "<< std::endl;
                addLayerTuple(std::to_string(0));
                //gridmap.lock();
                if (behaviour==1){
                    std::string path_frame = path.header.frame_id;
                    to_global_transform = tf_buffer_.lookupTransform(map_frame_, path_frame, ros::Time::now(), ros::Duration(0.1) );
                }

                int c = 0;
                for (auto p : path.poses){
                    if (behaviour==1){
                        convert(p.pose);
                    }
                    position(0) = p.pose.position.x;
                    position(1) = p.pose.position.y;
                    gridmap.gridmap.getIndex(position, index);

                    //TODO UPDATE
                    gridmap.gridmap.at(std::to_string(0), index) = std::max(static_cast<double>(gridmap.gridmap.at(std::to_string(0), index)),exp(-0.005*c));
                    if (gridmap.gridmap.at(std::to_string(0), index) < 0){
                        gridmap.gridmap.at(std::to_string(0), index) = std::max(static_cast<double>(gridmap.gridmap.at(std::to_string(0),index)),0.1*c);
                    }

                    c++;
                }
                //gridmap.setDataFlag(true);  
                //gridmap.unlock();
            }
            
            LayerSubscriber(const LayerSubscriber& other): tf2_listener_(tf_buffer_), nh_(), local_frame_(other.local_frame_), 
                                                            map_frame_(other.local_frame_), search_depth_(other.search_depth_),
                                                            is_local_(other.is_local_), resolution_(other.resolution_){
                topic_ = other.topic_;
                ops.topic = topic_;//"/" + input;//options_.rate_control_topic;
                ops.queue_size = 1;
                ops.md5sum = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
                ops.datatype = ros::message_traits::datatype<topic_tools::ShapeShifter>();
                ops.helper = boost::make_shared<ros::SubscriptionCallbackHelperT<
                        const ros::MessageEvent<topic_tools::ShapeShifter const> &> >(
                                boost::bind(&LayerSubscriber::layerCB, this, _1));
                rsub_ = nh_.subscribe(ops);
            }
            
            //do not modify local_frame ("frame of the messages of the persons" or get it from the message it self)
            LayerSubscriber(std::string input, double resolution, bool local, int desired_depth=5, std::string map_frame="odom"): tf2_listener_(tf_buffer_), nh_(), search_depth_(desired_depth), 
                                                                                                            local_frame_("velodyne"), map_frame_(map_frame), is_local_(local),
                                                                                                            resolution_(resolution){

                rpub_ = nh_.advertise<geometry_msgs::PoseArray>("feedback", 1);
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
            ros::Publisher rpub_; 
            geometry_msgs::PoseArray fb_msgs_;
            geometry_msgs::TransformStamped to_global_transform;
            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf2_listener_;
            boost::mutex mt;
            ros::SubscribeOptions ops;
            int search_depth_;
            std::string local_frame_;
            std::string map_frame_;
            bool is_local_;
            double resolution_;

    };
};


