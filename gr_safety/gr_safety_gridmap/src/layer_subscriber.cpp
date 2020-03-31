#include <gr_safety_gridmap/layer_subscriber.h>

using namespace gr_safety_gridmap;

void LayerSubscriber::layerCB(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event){
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

std::string LayerSubscriber::getLayerId(){
    return id_;
}

void LayerSubscriber::reset(){
    gridmap.add(id_, 0);
}

void LayerSubscriber::updateLayer(const geometry_msgs::PoseArray& poses, int behaviour){
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

void LayerSubscriber::updateLayer(const nav_msgs::Path& path, int behaviour){
    //boost::shared_ptr<grid_map::GridMap> pmap;
    grid_map::Position position;
    grid_map::Index index;
    int c = 0;  
    for (auto p : path.poses){
        position(0) = p.pose.position.x;
        position(1) = p.pose.position.y;
        gridmap.getIndex(position, index);
        gridmap.at(id_, index) = std::max(static_cast<double>(gridmap.at(id_, index)),exp(-0.005*c));
        c++;
    }
}

LayerSubscriber::LayerSubscriber(){

}

LayerSubscriber::LayerSubscriber(const LayerSubscriber& other){
    id_ = other.id_;
    rsub_ = other.rsub_;
}

LayerSubscriber::LayerSubscriber(std::string input, std::string id): id_(id){
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
