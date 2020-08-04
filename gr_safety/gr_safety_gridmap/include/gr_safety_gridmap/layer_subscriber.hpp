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
#include <angles/angles.h>

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

                    if (o.is_dynamic){
                        updateDynamic(o);
                    }
                    else{
                        updateStatic(o);
                    }
                }
                rpub_.publish(fb_msgs_);
                //gridmap.setDataFlag(true);
                };
            }


            void updateDynamic(safety_msgs::Object o){
                auto currentpose = o.pose;
                auto aux = currentpose;
                //float ospeed = sqrt(pow(o.speed.x,2) + pow(o.speed.y,2));
                int searchdepth = tracking_time_;//int(tracking_distance_/(nprimitives_*ospeed));
                //recursivity
                search_depth_ = searchdepth;
                generateCycle(aux,searchdepth, o.object_id, o.speed);
                    
                //Update current position
                convert(currentpose);
                updateGridLayer(o.object_id, currentpose,  exp(0));
            }

            void updateStatic(safety_msgs::Object o){
                grid_map::Position center;
                grid_map::Index index;
                auto currentpose = o.pose;
                convert(currentpose);
                center(0) = currentpose.position.x;
                center(1) = currentpose.position.y;
                
                bool validindex = gridmap.gridmap.getIndex(center, index);
                if (!validindex){
                    return;
                }

                for (grid_map::CircleIterator iterator(gridmap.gridmap, center, proxemic_distance_);!iterator.isPastEnd(); ++iterator) {
                    gridmap.gridmap.at(o.object_id, *iterator) = 0.01;
                }
            }

            void generateCycle(geometry_msgs::Pose in, int depth, std::string layer, const geometry_msgs::Vector3 v){
                if (depth == 0){
                    return;
                }
                //generateCycle(in,depth);
                geometry_msgs::Pose aux, aux2;
                float radius = 1.0;
                aux = in;
                //MotionModel class TODO
                double prob[9] ={1.0,1.0,1.0,1.0,1.0,0.5,0.5,0.05,0.05};
                //auto norm = nprimitives*exp(-0.3*(search_depth_-depth));

                for (int i=0; i <nprimitives_; i++){
                    aux2 = generateMotion(in,i, v);
                    //to mapframe
                    convert(aux2);
                    auto val = 2*exp(-0.3*(search_depth_-depth))*prob[i];
                    //val /=norm;

                    if (updateGridLayer(layer, aux2, val )){
                        fb_msgs_.poses.push_back(aux2);
                    }
                    generateCycle(aux2,depth-1,layer, v);

                }
            }


            bool updateGridLayer(const std::string layer_id, geometry_msgs::Pose p, double val){
                /*
                grid_map::Position position;
                grid_map::Index index;

                position(0) = p.position.x;
                position(1) = p.position.y;
                bool validindex = gridmap.gridmap.getIndex(position, index);
                if (!validindex){
                    //ROS_WARN_STREAM("index not valid"<<index<< map_frame_);
                    return false;
                }
                //gridmap.gridmap.at(layer_id, index) += val;
                gridmap.gridmap.at(layer_id, index) = std::max(static_cast<double>(gridmap.gridmap.at(layer_id, index)),val);

                */
                grid_map::Position center;
                grid_map::Index index;
                center(0) = p.position.x;
                center(1) = p.position.y;
                

                /*
                bool validindex = gridmap.gridmap.getIndex(center, index);
                if (!validindex){
                    return;
                }
                */

                for (grid_map::CircleIterator iterator(gridmap.gridmap, center, resolution_);!iterator.isPastEnd(); ++iterator) {
                    gridmap.gridmap.at(layer_id, *iterator) =  std::max(static_cast<double>(gridmap.gridmap.at(layer_id, *iterator)),val);;
                }

                return true;
            }

            //TODO create MotionModelClass
            geometry_msgs::Pose generateMotion(const geometry_msgs::Pose in, int motion_type, const geometry_msgs::Vector3 ov){
                //this motion is a hack... motion on the sensor frame not the relative path
                geometry_msgs::Pose out;
                out = in;

                //double dt = (current_time - last_time).toSec();
                auto dt = 1;
                float vx =0.0;
                float vy =0.0;

                auto th = tf2::getYaw(in.orientation);
                double delta_th = ov.z*0.1;// * dt;

                switch(motion_type){
                    case 0:
                        //th += delta_th;
                        vx = fabs(ov.x);
                        //vx = std::max(resolution_,2.0);
                        break;
                    case 1:
                        vx = fabs(ov.x);
                        //vx = std::max(resolution_,2.0);
                        th += delta_th;
                        break;
                    case 2:
                        vx = fabs(ov.x);
                        //vx = std::max(resolution_,2.0);
                        th -= delta_th;
                        break;
                    case 3:
                        vx = fabs(ov.x);
                        //vx = std::max(resolution_,2.0);
                        th -= M_PI+delta_th;
                        break;
                    case 4:
                        vx = fabs(ov.x);
                        //vx = std::max(resolution_,2.0);
                        th += M_PI+delta_th;
                        break;
                    case 5:
                        vx = -fabs(ov.x);
                        //vx = -std::max(resolution_,2.0);
                        //vy = 1;
                        //th -= delta_th;
                        break;
                    case 6:
                        //out.position.x = in.position.x-resolution_;
                        //out.position.y = in.position.y+resolution
                        th += delta_th;
                        break;
                    case 7:
                        //out.position.x = in.position.x+resolution_;
                        //out.position.y = in.position.y-resolution_;
                        th -= delta_th;
                        break;
                    case 8:                     //out = in;
                        break;
                }

                th = angles::normalize_angle(th);

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
                                                            is_local_(other.is_local_), resolution_(other.resolution_),
                                                            tracking_time_(other.tracking_time_), nprimitives_(other.nprimitives_),
                                                            proxemic_distance_(other.proxemic_distance_){
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
            LayerSubscriber(std::string input, double resolution, bool local, int tracking_time=2.0, int nprimitives=3, float proxemic_distance=5.0, std::string map_frame="odom"): tf2_listener_(tf_buffer_), nh_(), 
                                                                                                            search_depth_(3), local_frame_("velodyne"), map_frame_(map_frame), is_local_(local),
                                                                                                            resolution_(resolution), tracking_time_(tracking_time),nprimitives_(nprimitives),
                                                                                                            proxemic_distance_(proxemic_distance){

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
            int nprimitives_;
            std::string local_frame_;
            std::string map_frame_;
            bool is_local_;
            float tracking_time_;
            double resolution_;
            float proxemic_distance_;

    };
};


