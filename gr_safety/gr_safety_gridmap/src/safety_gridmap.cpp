#include<gr_safety_gridmap/safety_gridmap.h>

using namespace gr_safety_gridmap;

SafetyGridMap::SafetyGridMap(){

    layer_subscribers.insert(std::pair<std::string, LayerSubscriber<nav_msgs::Path>>("a", LayerSubscriber<nav_msgs::Path>()));
}

void SafetyGridMap::updateGrid(){
    boost::shared_ptr<grid_map::GridMap> pmap = boost::make_shared<grid_map::GridMap>(cmap_);

    for (auto& it : layer_subscribers){
        if (it.second.isMessageReceived()){
            it.second.updateLayer(pmap);
            ROS_INFO_STREAM(pmap->exists(it.first) << " MAIN ");
        }
    }
}
