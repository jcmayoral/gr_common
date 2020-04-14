#include<gr_safety_gridmap/safety_gridmap.h>

using namespace gr_safety_gridmap;

SafetyGridMap::SafetyGridMap(){
    bool local_gridmap = true;
    double resolution = 0.2;

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
        ROS_INFO_STREAM("Working");
        
    gridmap.gridmap.setFrameId(map_frame);
    gridmap.gridmap.setGeometry(grid_map::Length(map_size*factor, map_size*factor), resolution);
    addStaticLayer("safety_regions");
     
    
    std::vector<std::string> layers;
    layers = gridmap.gridmap.getLayers();

    for (auto l: layers){
        std::cout << "layers aftert" <<  l << std::endl;
    }
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


void SafetyGridMap::addStaticLayer(std::string iid){
    boost::mutex::scoped_lock(gridmap.mtx);
    if(!gridmap.gridmap.exists(layer_id)){
        ROS_ERROR_STREAM("ADD static:"<< iid);
        gridmap.gridmap.add(iid, grid_map::Matrix::Random(gridmap.gridmap.getSize()(0), gridmap.gridmap.getSize()(1)));
      }
      /*
      std::vector<std::string> layers;
      layers = gridmap.gridmap.getLayers();

      for (auto l: layers){
        std::cout << "layers" <<  l << " map " << gridmap.id << std::endl;
      }
      */
  };
}

void SafetyGridMap::updateGrid(){
    //modifications on this pointer get lost when function dies.
    //boost::shared_ptr<grid_map::GridMap> pmap = boost::make_shared<grid_map::GridMap>(cmap_);
    boost::mutex::scoped_lock ltk(gridmap.mtx);{
        //std::cout << "Update " << gridmap.id << std::endl;
        if ( gridmap.gridmap.exists("Trajectory_0") &&  gridmap.gridmap.exists("Trajectory_1")){
            gridmap.gridmap.add("sum",  gridmap.gridmap.get("Trajectory_0") +  gridmap.gridmap.get("Trajectory_1"));
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
