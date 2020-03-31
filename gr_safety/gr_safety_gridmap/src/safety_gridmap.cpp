#include<gr_safety_gridmap/safety_gridmap.h>

using namespace gr_safety_gridmap;

SafetyGridMap::SafetyGridMap(){
    boost::mutex::scoped_lock(gridmap.mtx);
    {
    gridmap.gridmap.setFrameId("map");
    gridmap.gridmap.setGeometry(grid_map::Length(10.0, 10.0), 0.1);
    }
    ros::NodeHandle nh;
    rpub_ = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    //TODO config file
    layer_subscribers.emplace_back("move_base/NavfnROS/plan", "layer_0");
    layer_subscribers.emplace_back("pcl_gpu_tools/detected_objects", "layer_1");
    //layer_subscribers.emplace_back("move_base/NavfnROS/plan", "layer_1");

    //layer_subscribers.emplace_back("input", "layer_name");
    //layer_subscribers.emplace_back("pcl_gpu_tools/detected_objects", "layer_1");
}

void SafetyGridMap::publishGrid(){
    gridmap.gridmap.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    boost::mutex::scoped_lock ltk(gridmap.mtx);{
    grid_map::GridMapRosConverter::toMessage(gridmap.gridmap, message);
    };
    rpub_.publish(message);
}

void SafetyGridMap::updateGrid(){
    //modifications on this pointer get lost when function dies.
    //boost::shared_ptr<grid_map::GridMap> pmap = boost::make_shared<grid_map::GridMap>(cmap_);
    
    for (auto& it : layer_subscribers){
       // std::cout << it.second.isMessageReceived();
        //auto layers = gridmap.gridmap.getLayers();

        /*
        for (auto l : layers){
            ROS_INFO_STREAM(" layer "<< l);
        }
        */
    }
}
