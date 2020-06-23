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

void SafetyGridMap::timer_callback(const ros::TimerEvent& event){
    boost::mutex::scoped_lock lltk(smtx);
    boost::mutex::scoped_lock ltk(gridmap.mtx);
    //updateGrid();
    //Should I clear just obstacle layers
    //gridmap.gridmap.clearAll();
    //reload static
    
    std::string obstacle_layer(":Prediction");
    std::string mask_layer(":Mask");
    std::string person_id;

    auto layers =  gridmap.gridmap.getLayers();
    for (auto l :  layers){
        if (l.find(":") == std::string::npos) {
            continue;
        }
        person_id = l.substr(0,l.find(":"));
        std::map<std::string,ros::Time>::iterator it;
        it = update_times_.find(person_id);

        if (it == update_times_.end()){
            update_times_[person_id] = ros::Time::now();                        
            continue;
        }

        auto transcurred_time = (ros::Time::now() - update_times_[person_id]).toSec();
        std::cout << "transcurred time since update " << transcurred_time << " id " << person_id << std::endl;
        if (transcurred_time < 5.0){
             update_times_[person_id] = ros::Time::now();  
             continue;
        }

        ROS_ERROR_STREAM("Erase "<< person_id);
       
        if(gridmap.gridmap.exists(person_id+obstacle_layer)){
            gridmap.gridmap.erase(person_id+obstacle_layer);
        }
        //gridmap.gridmap[person_id+obstacle_layer].setZero();
        //gridmap.gridmap[person_id+mask_layer].setZero();
        if(gridmap.gridmap.exists(person_id+mask_layer)){
            gridmap.gridmap.erase(person_id+mask_layer);
        } 
        update_times_.erase(it);
    }
}

void SafetyGridMap::initializeGridMap(bool localgridmap){
    ros::NodeHandle nh;

    //LOAD FROM gridmap_config.yaml
    std::string path = ros::package::getPath("gr_safety_gridmap");
    std::string config_path;
    std::string config_file;
    nh.param<std::string>("config_path", config_path, "config");
    nh.param<std::string>("config_file", config_file, "gridmap_config.yaml");
    YAML::Node config_yaml = YAML::LoadFile((path+"/"+config_path+"/"+config_file).c_str());

    ROS_ERROR_STREAM("USE LOCAL GRIDMAP: "<< localgridmap);
    auto resolution = config_yaml["resolution"].as<double>();
    ROS_ERROR_STREAM("RESOLUTION: "<< resolution);
    auto map_size = config_yaml["mapsize"].as<double>();
    ROS_ERROR_STREAM("MAP SIZE: "<< map_size);

    std::string map_frame;
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
    //TODO INTEGRATE DYNAMIC SAFETY REGIOS FOR GLOBAL GRIDMAP
    addStaticLayer("safetyregions");
    loadRegions("safetyregions");
    
    /*
    std::vector<std::sting> blayers;
    blayers.push_back("safetyregions");
    gridmap.gridmap.setBasicLayers(blayers);
    */
    }

    rpub_ = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    safety_grader_ = nh.advertise<std_msgs::Float32>("safety_score", 1, true);

    // Transformin the entire path of location can be computaitonal expensive
    if(!localgridmap){
        auto pathtopic = config_yaml["pathtopic"].as<std::string>();
        layer_subscribers.emplace_back(pathtopic.c_str(), resolution, localgridmap, map_frame);
    }

    const YAML::Node& detection_topics = config_yaml["detection_topics"];
    std::cout << "Number of Obstacle Topics to Subscribe " << detection_topics.size() << std::endl;

    for (YAML::const_iterator it= detection_topics.begin(); it != detection_topics.end(); it++){
      std::string topic = it->as<std::string>();
      ROS_INFO_STREAM("Subscribing to " << topic);
      layer_subscribers.emplace_back(topic.c_str(), resolution, localgridmap, map_frame);
    }

    clear_timer_ = nh.createTimer(ros::Duration(5.0), &SafetyGridMap::timer_callback, this);
    //ros::spin();
}

void SafetyGridMap::publishGrid(){
    gridmap.gridmap.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    //boost::mutex::scoped_lock ltk(gridmap.mtx);{
    grid_map::GridMapRosConverter::toMessage(gridmap.gridmap, message);
    //};
    rpub_.publish(message);
}


void SafetyGridMap::addStaticLayer(std::string iid){
    //boost::mutex::scoped_lock(gridmap.mtx);
    if(!gridmap.gridmap.exists(iid)){
        gridmap.gridmap.add(iid, 0);//grid_map::Matrix::Random(gridmap.gridmap.getSize()(0), gridmap.gridmap.getSize()(1)));
    }
    //clear safetyregions just in case
    gridmap.gridmap[iid].setZero();
}


void SafetyGridMap::loadRegions(std::string iid){
    std::string path = ros::package::getPath("gr_safety_gridmap");
    YAML::Node config_yaml = YAML::LoadFile((path+"/config/safety_regions.yaml").c_str());
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
    boost::mutex::scoped_lock lltk(smtx);
    boost::mutex::scoped_lock ltk(gridmap.mtx);
        //if (!gridmap.isNewDataAvailable()){
            //ROS_ERROR_STREAM("Waiting for new data");
          //  return;
        //}
        //analyze if log or probability can do a better approach
        //if(!gridmap.gridmap.exists("conv")){
        //gridmap.gridmap.add("conv", 0);//grid_map::Matrix::Random(gridmap.gridmap.getSize()(0), gridmap.gridmap.getSize()(1)));
        //}
        gridmap.gridmap.add("conv", 0);
        auto safety_layer = gridmap.gridmap.get("safetyregions");
        auto layers =  gridmap.gridmap.getLayers();

        std::string obstacle_layer(":Prediction");
        std::string mask_layer(":Mask");
        std::string person_id;
        int nobjects = 0;

        for (auto l :  layers){
            //std::cout << l << std::endl;
            if (l.find(":") == std::string::npos) { //I prefere this than storing objects id... could be messy
                //ROS_ERROR_STREAM("skip "<< l<< ", "<< l.find(mask_layer));
                continue;
            }
            if (l.find(mask_layer) == std::string::npos) { //skip time mask to process object nu
                //ROS_ERROR_STREAM("skip mask "<< l);
                continue;
            }
            //std::cout << l <<  ", " << l.find(mask_layer) + 1 << std::endl;
            person_id = l.substr(0,l.find(":")); 

            //TODO fix this`
            if (person_id.empty()){
                continue;
            }

            if(!gridmap.gridmap.exists(person_id+obstacle_layer)){
                std::cout << "avoid "<<std::endl;
                continue;
            }
            std::cout << person_id << " layer has been read " << std::endl;
            auto layer = gridmap.gridmap.get(person_id+obstacle_layer);
            auto timed_mask = gridmap.gridmap.get(person_id+mask_layer);
            gridmap.gridmap["conv"] = gridmap.gridmap.get("conv") + (timed_mask.array() * layer.array()).matrix();// * timed_mask;
            nobjects++;
        }
        std_msgs::Float32 score;
        gridmap.gridmap["conv"] = gridmap.gridmap.get("conv").cwiseProduct(gridmap.gridmap.get("safetyregions"));// * safety_layer;
        //ROS_INFO_STREAM("debug this line after testing " << gridmap.gridmap["conv"].maxCoeff());
        score.data = gridmap.gridmap.get("conv").sum();

        ROS_ERROR_STREAM("risk sum " << score.data << " and nobjects processed " << nobjects);

        //Publish Score
        safety_grader_.publish(score);
        publishGrid();
        //ResetFlag
        //gridmap.setDataFlag(false);

}
