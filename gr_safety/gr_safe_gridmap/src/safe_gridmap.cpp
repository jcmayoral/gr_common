#include<gr_safe_gridmap/safe_gridmap.h>

using namespace gr_safe_gridmap;

SafeGridMap::SafeGridMap(){
    ROS_INFO_STREAM("Default local gridmap");
    bool localgridmap = true;
    initializeGridMap(localgridmap);
}

SafeGridMap::SafeGridMap(bool localgridmap){
    ROS_INFO_STREAM("Selecting gridmap mode "<< localgridmap);
    initializeGridMap(localgridmap);
}

void SafeGridMap::timer_callback(const ros::TimerEvent& event){
    boost::mutex::scoped_lock lltk(smtx);
    boost::mutex::scoped_lock ltk(gridmap.mtx);
    //updateGrid();
    //Should I clear just obstacle layers
    //gridmap.gridmap.clearAll();
    //reload static
    
    std::string object_id;

    auto layers =  gridmap.gridmap.getLayers();
    for (auto l :  layers){
        if (l.find("object_") == std::string::npos) {
            continue;
        }
        object_id = l.substr(l.find("object_")); 

        std::map<std::string,ros::Time>::iterator it;
        it = gridmap.update_times_.find(object_id);

        auto transcurred_time = (ros::Time::now() - gridmap.update_times_[object_id]).toSec();

        if (transcurred_time < 5.0){
             continue;
        }

        ROS_ERROR_STREAM("Erase "<< object_id);
       
        if(gridmap.gridmap.exists(object_id)){
            gridmap.gridmap.clear(object_id);
            gridmap.gridmap.erase(object_id);
        }

        if (it != gridmap.update_times_.end()){
            gridmap.update_times_.erase(it);
        }
    }
}

void SafeGridMap::initializeGridMap(bool localgridmap){
    ros::NodeHandle nh;

    //LOAD FROM gridmap_config.yaml
    std::string path = ros::package::getPath("gr_safe_gridmap");
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
    auto clearing_timeout = config_yaml["timeout"].as<double>();
    ROS_ERROR_STREAM("Clearing Timeout: "<< clearing_timeout);
    int tracking_time = config_yaml["trackingtime"].as<int>();
    ROS_ERROR_STREAM("Time to Track: "<< tracking_time);
    auto nprimitives = config_yaml["nprimitives"].as<int>();
    ROS_ERROR_STREAM("Number of Primitives: "<< nprimitives);

    float proxemicdistance = config_yaml["proxemicradius"].as<float>();
    ROS_ERROR_STREAM("Proxemic Distance: "<< proxemicdistance);

    safety_threshold_= config_yaml["safety_threshold"].as<double>();
    ROS_ERROR_STREAM("Threshold: "<< safety_threshold_);

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
    objects_risk_pub_ = nh.advertise<safety_msgs::RiskIndexes>("safety_indexes", 1, true);
    request_stop_ = nh.advertise<std_msgs::Bool>("/lock_all", 1, true);

    // Transformin the entire path of location can be computaitonal expensive
    if(!localgridmap){
        auto pathtopic = config_yaml["pathtopic"].as<std::string>();
        layer_subscribers.emplace_back(pathtopic.c_str(), resolution, localgridmap, tracking_time, nprimitives, proxemicdistance, map_frame);
    }

    const YAML::Node& detection_topics = config_yaml["detection_topics"];
    std::cout << "Number of Obstacle Topics to Subscribe " << detection_topics.size() << std::endl;

    for (YAML::const_iterator it= detection_topics.begin(); it != detection_topics.end(); it++){
      std::string topic = it->as<std::string>();
      ROS_INFO_STREAM("Subscribing to " << topic);
      layer_subscribers.emplace_back(topic.c_str(), resolution, localgridmap, tracking_time, nprimitives, proxemicdistance, map_frame);
    }

    clear_timer_ = nh.createTimer(ros::Duration(clearing_timeout), &SafeGridMap::timer_callback, this);
    //ros::spin();
}

void SafeGridMap::publishGrid(){
    gridmap.gridmap.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    //boost::mutex::scoped_lock ltk(gridmap.mtx);{
    grid_map::GridMapRosConverter::toMessage(gridmap.gridmap, message);
    //};
    rpub_.publish(message);
}


void SafeGridMap::addStaticLayer(std::string iid){
    //boost::mutex::scoped_lock(gridmap.mtx);
    if(!gridmap.gridmap.exists(iid)){
        gridmap.gridmap.add(iid, 0);//grid_map::Matrix::Random(gridmap.gridmap.getSize()(0), gridmap.gridmap.getSize()(1)));
    }
    //clear safetyregions just in case
    gridmap.gridmap[iid].setZero();
}


void SafeGridMap::loadRegions(std::string iid){
    std::string path = ros::package::getPath("gr_safe_gridmap");
    YAML::Node config_yaml = YAML::LoadFile((path+"/config/safety_regions.yaml").c_str());
    for (YAML::const_iterator a= config_yaml.begin(); a != config_yaml.end(); ++a){
        auto risk_level = a->first.as<float>();
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

void SafeGridMap::updateGrid(){
    boost::mutex::scoped_lock lltk(smtx);
    boost::mutex::scoped_lock ltk(gridmap.mtx);
    if (!gridmap.isNewDataAvailable()){
	    ROS_ERROR("NO UPDATES");
    return;	    
    }

        gridmap.gridmap.add("conv", 0);
        auto safety_layer = gridmap.gridmap.get("safetyregions");
        auto layers =  gridmap.gridmap.getLayers();

        std::string object_id;
        int nobjects = 0;
        safety_msgs::RiskIndexes indexes;

        for (auto l :  layers){
            if (l.find("object_") == std::string::npos) {
                continue;
            }

            object_id = l.substr(l.find("object_")); 

            if (object_id.empty()){
                continue;
            }


            if(!gridmap.gridmap.exists(object_id)){
                std::cout << "avoid "<<std::endl;
                continue;
            }
            auto layer = gridmap.gridmap.get(object_id);
            //std::cout << "MAX OBJECT LAYER g" << layer.maxCoeff()<<std::endl;
            // auto normlayer = layer/layer.maxCoeff();
            float object_risk_index = layer.cwiseProduct(safety_layer).maxCoeff();
            gridmap.gridmap["conv"] = gridmap.gridmap.get("conv") + layer;
            nobjects++;

            safety_msgs::RiskObject robj;
            robj.object_id = object_id;
            robj.risk_index =object_risk_index;
            indexes.objects.push_back(robj);
        }

        //Time predicionts
        int timecount = 0;
        std::string timeid{"Time_" + std::to_string(timecount)};

        safety_msgs::RiskObject tobj;

        while (gridmap.gridmap.exists(timeid)){
            tobj.object_id = timeid;
            auto tlayer = gridmap.gridmap.get(timeid);//.matrix();
            tobj.risk_index = tlayer.cwiseProduct(safety_layer).maxCoeff();
            indexes.objects.push_back(tobj);
            timecount++;
            timeid = "Time_" + std::to_string(timecount);
        }

        std_msgs::Float32 score;
        gridmap.gridmap["conv"] = gridmap.gridmap.get("conv").cwiseProduct(safety_layer);
        //ROS_INFO_STREAM("debug this line after testing " << gridmap.gridmap["conv"].maxCoeff());
        score.data = gridmap.gridmap.get("conv").maxCoeff();//sum();

        //ROS_ERROR_STREAM_THROTTLE(1,"risk sum " << score.data << " MEAN SAFETY " << score.data/nobjects << "MAX COEFF "<<gridmap.gridmap.get("conv").maxCoeff());


        std_msgs::Bool stop_request;
        if (score.data > safety_threshold_){
            stop_request.data = true;
        }

        request_stop_.publish(stop_request);
        //Publish Score
        safety_grader_.publish(score);
        objects_risk_pub_.publish(indexes);
        publishGrid();
	gridmap.setDataFlag(false);
}
