#include<gr_safety_gridmap/safety_gridmap.h>

using namespace gr_safety_gridmap;

SafetyGridMap::SafetyGridMap(){
    cmap_.setFrameId("map");
    cmap_.setGeometry(grid_map::Length(10.0, 10.0), 0.20);
    //DEBUG
    cmap_.add("debug", grid_map::Matrix::Random(cmap_.getSize()(0), cmap_.getSize()(1)));
    ros::NodeHandle nh;
    rpub_ = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    layer_subscribers.insert(std::pair<std::string, LayerSubscriber<nav_msgs::Path>>("TEST_KAYER", LayerSubscriber<nav_msgs::Path>("TEST_KAYER")));
}

void SafetyGridMap::publishGrid(){
    cmap_.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(cmap_, message);
    rpub_.publish(message);

}

void SafetyGridMap::updateGrid(){
    //modifications on this pointer get lost when function dies.
    //boost::shared_ptr<grid_map::GridMap> pmap = boost::make_shared<grid_map::GridMap>(cmap_);
    for (auto& it : layer_subscribers){
       // std::cout << it.second.isMessageReceived();

        if (it.second.isMessageReceived()){
            std::cout <<"MSG Received"<<std::endl;
            it.second.updateLayer(cmap_);
            ROS_INFO_STREAM(cmap_.exists(it.first) << " MAIN ");
        }
        auto layers = cmap_.getLayers();

        for (auto l : layers){
            ROS_INFO_STREAM(" layer "<< l);
        }
    }
}
