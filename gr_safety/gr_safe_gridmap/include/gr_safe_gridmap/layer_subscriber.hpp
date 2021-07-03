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

#include <gr_safe_gridmap/main_gridmap.hpp>
#include <angles/angles.h>


#include <gr_safe_gridmap/SafeGridMapConfig.h>

//Based originally from the rosbag c++ implementation
//https://github.com/ros/ros_comm/tree/noetic-devel/tools/rosbag/src
//used as tutorial to get into the definition of a message.
extern gr_safe_gridmap::MainGrid gridmap;

namespace gr_safe_gridmap{
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
                //std::cout << msgtype << std::endl;
                if (type_.find("path") != std::string::npos) {
//                try{
                    std::cout << "PATH FOUND "<< std::endl;
                    nav_msgs::Path path;
                    path = *ssmsg->instantiate<nav_msgs::Path>();
                    //local gridmap -> 1
                    updateLayer(path, 1);
                    return;
                }
  //              }
    //            catch(...){

      //          }

               //try{
                std::cout << "TYPE " << std::endl;
                std::cout << type_ << std::endl;
                std::cout << "topic " << std::endl;
                std::cout << topic_ << std::endl;
                
                if (type_.find("obstacles") != std::string::npos) {
                    std::cout << "obstacles FOUND "<< std::endl;
                    safety_msgs::FoundObjectsArray parr;
                    parr = *ssmsg->instantiate<safety_msgs::FoundObjectsArray>();
                    updateLayer(parr, 1);
                    return;

                }
                //}

                //catch(...){
                ROS_ERROR_STREAM("Some erro processed -> TO DO SOMETHING TO STOP "<< type_);
                //}

            }

            void addLayerTuple(std::string person_id){
                gridmap.gridmap.add(person_id, 0.0);
            }

            void convert(geometry_msgs::PoseStamped& in){
                if (in.header.frame_id.compare(map_frame_)==0){
                    //std::cout << "AAA : " << in.header.frame_id << " >>> " << map_frame_ << std::endl;
                    return;
                }
                tf2::doTransform(in, in, to_global_transform);
            }

            void updateLayer(const safety_msgs::FoundObjectsArray& poses, int behaviour){
                //boost::mutex::scoped_lock lc(mt);
                //while (!gridmap.isNewDataAvailable());
                grid_map::Position position;
                grid_map::Index index;

                float radius = 0.5;
                //to_global_transform = tf_buffer_.lookupTransform(map_frame_, poses.header.frame_id, ros::Time(0), ros::Duration(0.5) );

                boost::mutex::scoped_lock lck(gridmap.mtx);{
                for (int i = 0; i <=tracking_time_; i++){
                    //gridmap.gridmap.clear("Time_"+std::to_string(i));
                    //std::cout << "LAYER  " << i << std::endl;
                    gridmap.gridmap["Time_"+std::to_string(i)].setZero();
                }

                fb_msgs_.header.frame_id = map_frame_;
                fb_msgs_.poses.clear();

                //index 0 reserved to robot
                for (auto o : poses.objects){
                    addLayerTuple(o.object_id);
                    gridmap.update_times_[o.object_id] = ros::Time::now();

                    if (o.is_dynamic){
                        updateDynamic(o, poses.header.frame_id);
                    }
                    else{
                        updateStatic(o, poses.header.frame_id);
                    }
                }
                rpub_.publish(fb_msgs_);
                last_reading_ = ros::Time::now();
                gridmap.setDataFlag(true);
                };
            }

            void updateDynamic(safety_msgs::Object o, const std::string frameid){
                geometry_msgs::PoseStamped currentpose;
                currentpose.header.frame_id = frameid;
                currentpose.pose = o.pose;
                auto aux = currentpose;
                //float ospeed = sqrt(pow(o.speed.x,2) + pow(o.speed.y,2));
                int searchdepth = tracking_time_;//int(tracking_distance_/(nprimitives_*ospeed));
                //recursivity
                search_depth_ = searchdepth;
                generateCycle(aux,searchdepth, o.object_id, o.speed);

                //Update current position
                convert(currentpose);
                updateGridLayer(o.object_id, currentpose,  exp(0), 0);
            }

            void updateStatic(safety_msgs::Object o, const std::string frameid){
                grid_map::Position center;
                grid_map::Index index;
                geometry_msgs::PoseStamped currentpose;
                currentpose.header.frame_id = frameid;
                currentpose.pose = o.pose;
                convert(currentpose);
                center(0) = currentpose.pose.position.x;
                center(1) = currentpose.pose.position.y;

                bool validindex = gridmap.gridmap.getIndex(center, index);
                if (!validindex){
                   return;
                }

                for (grid_map::CircleIterator iterator(gridmap.gridmap, center, proxemic_distance_);!iterator.isPastEnd(); ++iterator) {
                    gridmap.gridmap.at(o.object_id, *iterator) = 1.0;
                }

                for (grid_map::CircleIterator iterator(gridmap.gridmap, center, proxemic_distance_);!iterator.isPastEnd(); ++iterator) {
                    for (int i = 0; i < tracking_time_; i++){
                        gridmap.gridmap.at("Time_"+std::to_string(i), *iterator) = 1.0;//0.1*(tracking_time_-i);
                    }
                }
            }

            void generateCycle(geometry_msgs::PoseStamped in, int depth, std::string layer, const geometry_msgs::Vector3 v){
                if (depth == 0){
                    return;
                }
                //generateCycle(in,depth);
                geometry_msgs::PoseStamped aux, aux2;
                float radius = 1.0;
                aux = in;
                //MotionModel class TODO MARKOV
                //double prob[9] ={0.5,0.25,0.25,0.05,0.05,0.05,0.05,0.05,0.05};
                //auto norm = nprimitives_*exp(-0.3*(search_depth_-depth));
                geometry_msgs::Pose p;

                for (int i=0; i <nprimitives_; i++){
                    //to mapframe
                    aux2 = generateMotion(in,i, v);

                    //if (aux2.pose.position.z > 1.1){
                    //    std::cout << "GENCY " << depth << layer << std::endl;
                    //}

                    float val = 1*exp(-0.5*(search_depth_-depth))*motion_prob_[i];
                    //val /=norm;
                    if (val > 1.0){
                        std::cout << "WTF WHYYYY "<<std::endl;
                        val = 1.0;
                    }

                    if (updateGridLayer(layer, aux2, val, 1+search_depth_-depth)){
                        p = aux2.pose;
                        fb_msgs_.poses.push_back(p);
                    }
                    generateCycle(aux2,depth-1,layer, v);

                }
            }

            bool updateGridLayer(const std::string layer_id, geometry_msgs::PoseStamped p, float val, const int timeindex){
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

                center(0) = p.pose.position.x;
                center(1) = p.pose.position.y;


                /*
                bool validindex = gridmap.gridmap.getIndex(center, index);
                if (!validindex){
                    return;
                }
                */

                std::string layname{"Time_"+std::to_string(timeindex)};
                //std::cout << "update " << layname << std::endl;

                for (grid_map::CircleIterator iterator(gridmap.gridmap, center, resolution_*2 );!iterator.isPastEnd(); ++iterator) {
                    gridmap.gridmap.at(layer_id, *iterator) = std::max( gridmap.gridmap.at(layer_id, *iterator) ,val);
                    gridmap.gridmap.at(layname, *iterator) =  std::max( gridmap.gridmap.at(layname, *iterator) ,val);
                }

                return true;
            }

            //TODO create MotionModelClass
            geometry_msgs::PoseStamped generateMotion(const geometry_msgs::PoseStamped in, int motion_type, const geometry_msgs::Vector3 ov){
                //this motion is a hack... motion on the sensor frame not the relative path

                geometry_msgs::PoseStamped out;
                out = in;

                convert(out);

                //every step should be a second
                //double dt = 0.5;//(ros::Time::now() - last_reading_).toSec();
                //auto dt = 10;
                float vx =0.0;
                float vy =0.0;

                auto th = tf2::getYaw(in.pose.orientation);
                double delta_th = ov.z*dt_;//ov.z*0.1;// * dt;

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

                //vx = vx;// + 3*resolution_;

                tf2::Quaternion quat_tf;
                double delta_x = (vx * cos(th) - vy * sin(th)) * dt_;
                double delta_y = (vx * sin(th) + vy * cos(th)) * dt_;
                out.pose.position.x = in.pose.position.x+delta_x;
                out.pose.position.y = in.pose.position.y+delta_y;
                quat_tf.setEuler(0,0,th);

                //std::cout << th << " normalize?" << std::endl;
                //tf2::fromMsg(in.orientation, quat_tf);
                out.pose.orientation = tf2::toMsg(quat_tf.normalize());

                return out;
            }

            void reconfigure(SafeGridMapConfig& config){
                boost::mutex::scoped_lock lck(layermtx);
                std::cout << "DYN " << subscriber_id_ << std::endl;
                if (config.selection.compare(subscriber_id_)!=0){
                    return;
                }
                switch (config.mode){
                    case 0:
                        ROS_WARN_STREAM("reconfig" << subscriber_id_);
                        setnPrimitive(config.nprimitives);
                        setTimeStep(config.timestep);
                        setTrackingSamples(config.tracking);
                        motion_prob_[0]= config.primitive_0;
                        motion_prob_[1] = config.primitive_1;
                        motion_prob_[2] = config.primitive_2;
                        motion_prob_[3] = config.primitive_3;
                        motion_prob_[4] = config.primitive_4;
                        motion_prob_[5] = config.primitive_5;
                        motion_prob_[6] = config.primitive_6;
                        motion_prob_[7] = config.primitive_7;
                        motion_prob_[8] = config.primitive_8;
                        break;
                    case 1:
                        ROS_WARN_STREAM("this must be implemented on safe gridmap" << subscriber_id_);
                        break;
                }
            }


            void updateLayer(const nav_msgs::Path& path, int behaviour){
                //boost::shared_ptr<grid_map::GridMap> pmap;
                boost::mutex::scoped_lock lck(gridmap.mtx);
                grid_map::Position position;
                grid_map::Index index;
                addLayerTuple("PATH");//std::to_string(0));
                //gridmap.lock();
                //if (behaviour==1){
                    std::string path_frame = path.header.frame_id;
                    std::cout << "convert from  " << path_frame << " to "<< map_frame_ <<std::endl;
                    to_global_transform = tf_buffer_.lookupTransform(map_frame_, path_frame, ros::Time::now(), ros::Duration(0.5) );
                    std::cout << "WWWW" <<std::endl;

                //}

                int c = 0;
                for (auto p : path.poses){
                    //if (behaviour==1){
                        std::cout << "call convert"<<std::endl;
                        convert(p);
                        std::cout << "CONVERT DONE << " <<p <<std::endl;
                    //}
                    position(0) = p.pose.position.x;
                    position(1) = p.pose.position.y;
                    gridmap.gridmap.getIndex(position, index);

                    //TODO UPDATE
                    gridmap.gridmap.at("PATH", index) = std::max(static_cast<double>(gridmap.gridmap.at("PATH", index)),exp(-0.005*c));
                    if (gridmap.gridmap.at("PATH", index) < 0){
                        gridmap.gridmap.at("PATH", index) = std::max(static_cast<double>(gridmap.gridmap.at("PATH",index)),0.1*c);
                    }

                    c++;
                }
                std::cout << "PROCESSED"<<std::endl;
                gridmap.setDataFlag(true);
                //gridmap.unlock();
            }

            LayerSubscriber(const LayerSubscriber& other): local_frame_(other.local_frame_), tf_buffer_(), subscriber_id_(other.subscriber_id_),
                                                            map_frame_(other.map_frame_), search_depth_(other.search_depth_),
                                                            is_local_(other.is_local_), resolution_(other.resolution_), dt_(other.dt_),
                                                            tracking_time_(other.tracking_time_), nprimitives_(other.nprimitives_),
                                                            type_(other.type_),
                                                            proxemic_distance_(other.proxemic_distance_), ops(other.ops),tf2_listener_(tf_buffer_),nh_(other.nh_){
                std::cout << "COPY CONSTRUCTOR "<< type_ << std::endl;
                topic_ = other.topic_;
                std::cout << "COPY CONSTRUCTOR topic "<< topic_ << std::endl;
                type_ = other.type_;
                last_reading_ = other.last_reading_;
                config();
            }

            std::string getTopic(){
                return topic_;
            }


            void config(){
                rpub_ = nh_.advertise<geometry_msgs::PoseArray>("feedback", 1);
                ops.topic = topic_;//"/" + input;//options_.rate_control_topic;
                ops.queue_size = 1;
                ops.md5sum = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
                ops.datatype = ros::message_traits::datatype<topic_tools::ShapeShifter>();
                ops.helper = boost::make_shared<ros::SubscriptionCallbackHelperT<
                        const ros::MessageEvent<topic_tools::ShapeShifter const> &> >(
                                boost::bind(&LayerSubscriber::layerCB, this, _1));
                rsub_ = nh_.subscribe(ops);
                ROS_INFO_STREAM("CONFIG"<< type_);
                ROS_INFO_STREAM("TOPIC "<< topic_);
                
            }

            //do not modify local_frame ("frame of the messages of the persons" or get it from the message it self)
            LayerSubscriber(std::string type, std::string id_nh, std::string topic, double resolution, bool local, int tracking_time=2, int nprimitives=3, float proxemic_distance=5.0, std::string map_frame="map"):
                                                                                                            tf_buffer_(),tf2_listener_(tf_buffer_), nh_(id_nh),
                                                                                                            search_depth_(3), local_frame_("velodyne"), map_frame_(map_frame), is_local_(local),
                                                                                                            resolution_(resolution), tracking_time_(tracking_time),nprimitives_(nprimitives),
                                                                                                            proxemic_distance_(proxemic_distance), dt_(0.5), motion_prob_{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
                                                                                                            type_(type)
            {
                std::cout << "constructor " << type_ << " INPUT "<< topic <<  std::endl;
                std::cout << " type " << type << std::endl;
                std::cout << "ID_NH " << id_nh <<std::endl;
                std::cout << "topic " << topic << std::endl;
                std::cout << "resolution " << resolution << std::endl;
                std::cout << "local " <<  local << std::endl;
                std::cout << "trackingtime " << tracking_time << std::endl;
                std::cout << "NPRIM " << nprimitives << std::endl;
                std::cout << "PROX" << proxemic_distance << std::endl;
                std::cout << "MAP FRAME " << map_frame << std::endl;
                subscriber_id_ = id_nh;
                last_reading_ = ros::Time::now();
                topic_ = topic;
                config();
            }

            void setTrackingSamples(int depth){
                tracking_time_ = depth;
            }

            void setTimeStep(double step){
                dt_ =step;
            }

            void setnPrimitive(int prims){
                nprimitives_ = prims;
            }

        private:
            boost::mutex layermtx;
            std::string topic_;
            boost::shared_ptr<ros::Subscriber> sub_;
            ros::NodeHandle nh_;
            ros::Subscriber rsub_;
            std::string type_;
            ros::Publisher rpub_;
            geometry_msgs::PoseArray fb_msgs_;
            geometry_msgs::TransformStamped to_global_transform;
            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf2_listener_;
            boost::mutex mt;
            ros::SubscribeOptions ops;
            int search_depth_;
            int nprimitives_;
            std::string subscriber_id_;
            std::string local_frame_;
            std::string map_frame_;
            bool is_local_;
            float tracking_time_;
            double resolution_;
            float proxemic_distance_;
            double dt_;
            ros::Time last_reading_;
            double motion_prob_[9];
    };
};
