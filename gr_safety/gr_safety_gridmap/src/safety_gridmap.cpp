#include<gr_safety_gridmap/safety_gridmap.h>

using namespace gr_safety_gridmap;

SafetyGridMap::SafetyGridMap(){
    bool local_gridmap = false;
    double resolution = 0.4;

    //odom->global base_link->local

    std::string map_frame;
    float map_size = 10.0;
    int factor;

    if (!local_gridmap){
        map_frame = "odom";
        factor = 10;
    }
    else{
        map_frame = "base_link";
        factor = 1;
    }
    
    boost::mutex::scoped_lock(gridmap.mtx);
    {
    gridmap.gridmap.setFrameId(map_frame);
    gridmap.gridmap.setGeometry(grid_map::Length(map_size*factor, map_size*factor), resolution);
    }
    ros::NodeHandle nh;
    rpub_ = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    //TODO config file
    layer_subscribers.emplace_back("move_base/NavfnROS/plan", resolution, local_gridmap, map_frame);
    layer_subscribers.emplace_back("pcl_gpu_tools/detected_objects", resolution, local_gridmap, map_frame);
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
    boost::mutex::scoped_lock ltk(gridmap.mtx);{

        if ( gridmap.gridmap.exists("layer_0") &&  gridmap.gridmap.exists("layer_1")){
            gridmap.gridmap.add("sum",  gridmap.gridmap.get("layer_0") +  gridmap.gridmap.get("layer_1"));
        }
    }
    
    //for (auto& it : layer_subscribers){
       // std::cout << it.second.isMessageReceived();
        //auto layers = gridmap.gridmap.getLayers();

        /*
        for (auto l : layers){
            ROS_INFO_STREAM(" layer "<< l);
        }
        */
    //}
}
