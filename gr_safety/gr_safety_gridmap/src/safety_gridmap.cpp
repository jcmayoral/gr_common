#include<gr_safety_gridmap/safety_gridmap.h>

using namespace gr_safety_gridmap;


SafetyGridMap::SafetyGridMap(){
    layer_subscribers.insert(std::pair<std::string, LayerSubscriber<nav_msgs::Path>>("a", LayerSubscriber<nav_msgs::Path>()));

}
