#include<gr_safety_gridmap/safety_gridmap.h>

using namespace gr_safety_gridmap;

SafetyGridMap::SafetyGridMap(){
    cmap_.setFrameId("map");
    cmap_.setGeometry(grid_map::Length(10.0, 10.0), 0.20);
    ros::NodeHandle nh;
    rpub_ = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    layer_subscribers.insert(std::pair<std::string, LayerSubscriber<nav_msgs::Path>>("aaaa", LayerSubscriber<nav_msgs::Path>("layer_0")));
}

void SafetyGridMap::publishGrid(){
    //cmap_.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(cmap_, message);
    rpub_.publish(message);

}

void SafetyGridMap::updateGrid(){
    boost::shared_ptr<grid_map::GridMap> pmap = boost::make_shared<grid_map::GridMap>(cmap_);
    for (auto& it : layer_subscribers){
       // std::cout << it.second.isMessageReceived();

        if (it.second.isMessageReceived()){
            std::cout <<"MSG Received"<<std::endl;
            it.second.updateLayer(pmap);
            ROS_INFO_STREAM(pmap->exists(it.first) << " MAIN ");
        }
    }
}
