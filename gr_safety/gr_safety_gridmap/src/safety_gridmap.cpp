#include<gr_safety_gridmap/safety_gridmap.h>

using namespace gr_safety_gridmap;

SafetyGridMap::SafetyGridMap(){
    gridmap.setFrameId("map");
    gridmap.setGeometry(grid_map::Length(10.0, 10.0), 0.20);
    ros::NodeHandle nh;
    rpub_ = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    layer_subscribers.emplace_back("test2");
    layer_subscribers.emplace_back("test");

}

void SafetyGridMap::publishGrid(){
    gridmap.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(gridmap, message);
    rpub_.publish(message);

}

void SafetyGridMap::updateGrid(){
    //modifications on this pointer get lost when function dies.
    //boost::shared_ptr<grid_map::GridMap> pmap = boost::make_shared<grid_map::GridMap>(cmap_);
    for (auto& it : layer_subscribers){
       // std::cout << it.second.isMessageReceived();
        auto layers = gridmap.getLayers();

        for (auto l : layers){
            ROS_INFO_STREAM(" layer "<< l);
        }
    }
}
