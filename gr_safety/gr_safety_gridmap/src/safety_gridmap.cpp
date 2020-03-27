#include<gr_safety_gridmap/safety_gridmap.h>

using namespace gr_safety_gridmap;

SafetyGridMap::SafetyGridMap():map_(new grid_map::GridMap({""})){
    layer_subscribers.insert(std::pair<std::string, LayerSubscriber<nav_msgs::Path>>("a", LayerSubscriber<nav_msgs::Path>(map_)));
}
