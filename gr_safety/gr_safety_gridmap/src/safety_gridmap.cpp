#include<gr_safety_gridmap/safety_gridmap.h>

using namespace gr_safety_gridmap;

SafetyGridMap::SafetyGridMap(){
    ROS_INFO_STREAM("Default local gridmap");
    bool localgridmap = true;
    initializeGridMap(localgridmap);
}

SafetyGridMap::SafetyGridMap(bool localgridmap){
    ROS_INFO_STREAM("Selecting gridmap mode "<< localgridmap);

    initializeGridMap(localgridmap);
}

void SafetyGridMap::initializeGridMap(bool localgridmap){
    //odom->global base_link->local
    double resolution = 1.0;

    std::string map_frame;
    float map_size = 30.0;
    int factor;

    if (!localgridmap){
        map_frame = "odom";
        factor = 3;
    }
    else{
        map_frame = "base_link";
        factor = 1;
    }
    
    boost::mutex::scoped_lock(gridmap.mtx);
    {   
    gridmap.gridmap.setFrameId(map_frame);
    gridmap.gridmap.setGeometry(grid_map::Length(map_size*factor, map_size*factor), resolution*factor);

    //TO Reduce complexity just applicable and storing coordinates of polygon on localframe
    //TODO apply tf transformation to polygon.
    if (localgridmap){
        addStaticLayer("safety_regions");
    }
    }

    ros::NodeHandle nh;
    rpub_ = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

    //TODO config file
    // Transformin the entire path of location can be computaitonal expensive
    if(!localgridmap){
        layer_subscribers.emplace_back("move_base/NavfnROS/plan", resolution, localgridmap, map_frame);
    }

    layer_subscribers.emplace_back("pcl_gpu_tools/detected_objects", resolution, localgridmap, map_frame);
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
    //boost::mutex::scoped_lock(gridmap.mtx);
    if(!gridmap.gridmap.exists(iid)){
        gridmap.gridmap.add(iid, 0);//grid_map::Matrix::Random(gridmap.gridmap.getSize()(0), gridmap.gridmap.getSize()(1)));
    }

    std::string path = ros::package::getPath("gr_safety_gridmap");
    YAML::Node config_yaml = YAML::LoadFile((path+"/config/safety_regions.yaml").c_str());

    //map_.clearAll();
    //clear safety_regions just in case
    gridmap.gridmap[iid].setZero();
    

    for (YAML::const_iterator a= config_yaml.begin(); a != config_yaml.end(); ++a){
        auto risk_level = a->first.as<int>();
        //std::cout << risk_level << std::endl;
        //create a polygon
        grid_map::Polygon polygon;
        polygon.setFrameId(gridmap.gridmap.getFrameId());

        //Load footprint as a list of pair x y
        std::list<std::pair<double, double>> markers = a->second.as<std::list<std::pair<double, double>>>();
        //add each tuple as a vertex
        for (auto& m : markers){
            polygon.addVertex(grid_map::Position(m.first, m.second));
        }
        //assign values in the gridmap
        for (grid_map::PolygonIterator iterator(gridmap.gridmap, polygon); !iterator.isPastEnd(); ++iterator) {
            gridmap.gridmap.at(iid, *iterator) = risk_level;
        }
    }
}

void SafetyGridMap::updateGrid(){
    boost::mutex::scoped_lock ltk(gridmap.mtx);{
        //analyze if log or probability can do a better approach
        gridmap.gridmap.add("conv", 0);//gridmap.gridmap.get("safety_regions"));
        auto safety_layer = gridmap.gridmap.get("safety_regions");

        /*
        for (auto l :  gridmap.gridmap.getLayers()){
            //ROS_INFO_STREAM(" layer "<< l);
            if (l.compare("safety_regions")==0|| l.compare("conv")==0){
                continue;
            }
            auto layer = gridmap.gridmap.get(l);
            gridmap.gridmap.add("conv", gridmap.gridmap.get("conv") - gridmap.gridmap.get("safety_regions") * layer);
        }
        */
        int person = 1;
        std::string obstacle_layer("Trajectory_"+std::to_string(person));
        std::string mask_layer("Mask_"+std::to_string(person));

        while (gridmap.gridmap.exists(obstacle_layer)){
            //std::cout << "person layer " << person << std::endl;
            auto layer = gridmap.gridmap.get(obstacle_layer);
            auto timed_mask = gridmap.gridmap.get(mask_layer);
            gridmap.gridmap["conv"] = gridmap.gridmap.get("conv") + timed_mask * layer;// * timed_mask;
            person++;
            obstacle_layer = "Trajectory_"+std::to_string(person);
            mask_layer = "Mask_"+std::to_string(person);

        }
        gridmap.gridmap["conv"] = gridmap.gridmap.get("conv").cwiseProduct(gridmap.gridmap.get("safety_regions"));// * safety_layer;
        ROS_INFO_STREAM("debug this line after testing " << gridmap.gridmap["conv"].maxCoeff());
    }
}
