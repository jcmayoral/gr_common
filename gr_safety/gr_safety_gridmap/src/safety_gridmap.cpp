#include<gr_safety_gridmap/safety_gridmap.h>

using namespace gr_safety_gridmap;


SafetyGridMap::SafetyGridMap(){
    //template T of LayerSubscriber currently not used
    layer_subscribers.insert(std::pair<std::string, LayerSubscriber<int>>("a", LayerSubscriber<int>()));

}
